#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace surf_drone {
// Discrete model at configured dt. State [delta debt, backlog debt]; input bytes/s.
class InformationAllocationController {
public:
  using Matrix = Eigen::Matrix2d;
  using State = Eigen::Vector2d;
  struct Model {
    Matrix a{Matrix::Identity()}, b{-0.001 * Matrix::Identity()};
    // Outdoor starting costs: preserve DELTA service against much larger BACKLOG.
    Matrix q{(Matrix() << 100.0, 0.0, 0.0, 1.0).finished()}, r{0.0001 * Matrix::Identity()};
  };
  struct Allocation {
    State requested{State::Zero()}, allocated{State::Zero()};
    double scale{1.0};
  };
  InformationAllocationController() {std::string error; update(Model{}, error);}
  bool update(const Model & candidate, std::string & error) {
    const auto & a = candidate.a; const auto & b = candidate.b;
    const auto & q = candidate.q; const auto & r = candidate.r;
    auto reject = [&](const char * why) {error = why; ++rejected_updates_; return false;};
    if (!a.allFinite() || !b.allFinite() || !q.allFinite() || !r.allFinite())
      return reject("nonfinite matrix");
    if (!q.isApprox(q.transpose(), 1e-10) || !r.isApprox(r.transpose(), 1e-10))
      return reject("Q/R must be symmetric");
    if (Eigen::SelfAdjointEigenSolver<Matrix>(q).eigenvalues().minCoeff() < -1e-12 ||
      Eigen::SelfAdjointEigenSolver<Matrix>(r).eigenvalues().minCoeff() <= 0)
      return reject("Q must be PSD and R positive definite");
    if (revision_ && a.isApprox(model_.a, 1e-10) && b.isApprox(model_.b, 1e-10) &&
      q.isApprox(model_.q, 1e-10) && r.isApprox(model_.r, 1e-10)) return true;
    Matrix p = q, k = Matrix::Zero();
    bool converged = false;
    for (int i = 0; i < 20000; ++i) {
      Matrix s = r + b.transpose() * p * b;
      if (!s.allFinite()) return reject("nonfinite Riccati input cost");
      Eigen::LDLT<Matrix> solve(s);
      if (solve.info() != Eigen::Success || !solve.isPositive())
        return reject("singular Riccati input cost");
      k = solve.solve(b.transpose() * p * a);
      Matrix next = q + a.transpose() * p * a - a.transpose() * p * b * k;
      next = (0.5 * (next + next.transpose())).eval();
      if (!next.allFinite()) return reject("nonfinite Riccati iterate");
      converged = (next - p).cwiseAbs().maxCoeff() <=
        1e-10 * std::max(1.0, next.cwiseAbs().maxCoeff());
      p = next;
      if (converged) break;
    }
    if (!converged) return reject("DARE did not converge");
    const Matrix input_cost = r + b.transpose() * p * b;
    if (!input_cost.allFinite()) return reject("nonfinite final input cost");
    k = input_cost.ldlt().solve(b.transpose() * p * a);
    if (!k.allFinite()) return reject("nonfinite gain");
    const Matrix closed_loop = a - b * k;
    if (!closed_loop.allFinite()) return reject("nonfinite closed-loop model");
    Eigen::EigenSolver<Matrix> eigen(closed_loop, false);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
      eigen.eigenvalues().cwiseAbs().maxCoeff() >= 1.0)
      return reject("unusable or non-stabilizing gain");
    model_ = candidate; gain_ = k; ++revision_; error.clear(); return true;
  }
  static Allocation project(State requested, double capacity) {
    Allocation out; out.requested = requested;
    for (int i = 0; i < 2; ++i)
      out.allocated[i] = std::isfinite(requested[i]) ? std::max(0.0, requested[i]) : 0.0;
    capacity = std::isfinite(capacity) ? std::max(0.0, capacity) : 0.0;
    const double largest = out.allocated.maxCoeff();
    if (largest > 0) {
      const State normalized = out.allocated / largest;
      if (largest > capacity / normalized.sum()) {
        out.scale = (capacity / largest) / normalized.sum();
        out.allocated = normalized * (capacity / normalized.sum());
      }
    }
    return out;
  }
  Allocation allocate(State debt, double capacity) const {
    for (int i = 0; i < 2; ++i)
      if (!std::isfinite(debt[i]) || debt[i] < 0) debt[i] = 0;
    return project(-gain_ * debt, capacity);
  }
  const Model & model() const {return model_;}
  const Matrix & gain() const {return gain_;}
  uint64_t revision() const {return revision_;}
  uint64_t rejected_updates() const {return rejected_updates_;}
private:
  Model model_; Matrix gain_{Matrix::Zero()};
  uint64_t revision_{0}, rejected_updates_{0};
};

// Conservative online identification from receiver-confirmed packet service.
// Debt transfers and priority changes are measured disturbances, so this model
// intentionally estimates only diagonal retention and service effectiveness.
class OnlineAllocationModelEstimator {
public:
  using Matrix = InformationAllocationController::Matrix;
  using State = InformationAllocationController::State;
  struct Config {
    bool enabled{false};
    double alpha{0.02};
    double minimum_effectiveness{0.001};
    double maximum_effectiveness{0.05};
    double minimum_retention{0.98};
    double maximum_retention{1.02};
    double maximum_relative_change{0.10};
    double update_interval_seconds{5.0};
    uint64_t minimum_ack_samples{20};
  };
  OnlineAllocationModelEstimator() = default;
  explicit OnlineAllocationModelEstimator(const Config & config) : config_(config) {}

  void observe_ack(int stream, std::size_t bytes, double priority) {
    if (stream < 0 || stream > 1 || bytes == 0 || !std::isfinite(priority) || priority <= 0) return;
    const double sample = std::clamp(
      priority / static_cast<double>(bytes), config_.minimum_effectiveness,
      config_.maximum_effectiveness);
    effectiveness_[stream] = ack_samples_[stream] == 0 ? sample :
      (1.0 - config_.alpha) * effectiveness_[stream] + config_.alpha * sample;
    ++ack_samples_[stream];
  }

  void observe_transition(
    const State & previous, const State & current, const State & disturbance,
    const State & acknowledged)
  {
    for (int stream = 0; stream < 2; ++stream) {
      if (!std::isfinite(previous[stream] + current[stream] + disturbance[stream] +
        acknowledged[stream]) || previous[stream] <= 1e-9) continue;
      const double sample = std::clamp(
        (current[stream] - disturbance[stream] + acknowledged[stream]) / previous[stream],
        config_.minimum_retention, config_.maximum_retention);
      retention_[stream] = transition_samples_[stream] == 0 ? sample :
        (1.0 - config_.alpha) * retention_[stream] + config_.alpha * sample;
      ++transition_samples_[stream];
    }
  }

  bool candidate(
    double now, double dt, const InformationAllocationController::Model & active,
    InformationAllocationController::Model & output)
  {
    if (!config_.enabled || !std::isfinite(now + dt) || dt <= 0 ||
      now - last_update_time_ < config_.update_interval_seconds ||
      ack_samples_[0] < config_.minimum_ack_samples ||
      ack_samples_[1] < config_.minimum_ack_samples ||
      transition_samples_[0] < config_.minimum_ack_samples ||
      transition_samples_[1] < config_.minimum_ack_samples ||
      ack_samples_ == samples_at_update_) return false;
    output = active;
    output.a.setZero(); output.b.setZero();
    for (int stream = 0; stream < 2; ++stream) {
      output.a(stream, stream) = bounded_change(
        active.a(stream, stream), retention_[stream]);
      output.b(stream, stream) = bounded_change(
        active.b(stream, stream), -dt * effectiveness_[stream]);
    }
    last_update_time_ = now;
    samples_at_update_ = ack_samples_;
    return true;
  }

  void accepted() {++updates_;}
  void rejected() {++rejections_;}
  const Config & config() const {return config_;}
  const std::array<uint64_t, 2> & ack_samples() const {return ack_samples_;}
  const std::array<uint64_t, 2> & transition_samples() const {return transition_samples_;}
  const std::array<double, 2> & effectiveness() const {return effectiveness_;}
  const std::array<double, 2> & retention() const {return retention_;}
  uint64_t updates() const {return updates_;}
  uint64_t rejections() const {return rejections_;}

private:
  double bounded_change(double active, double target) const {
    const double limit = std::max(std::abs(active) * config_.maximum_relative_change, 1e-9);
    return std::clamp(target, active - limit, active + limit);
  }
  Config config_;
  std::array<uint64_t, 2> ack_samples_{}, transition_samples_{}, samples_at_update_{};
  std::array<double, 2> effectiveness_{}, retention_{{1.0, 1.0}};
  double last_update_time_{-std::numeric_limits<double>::infinity()};
  uint64_t updates_{0}, rejections_{0};
};
}  // namespace surf_drone
