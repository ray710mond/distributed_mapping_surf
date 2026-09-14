#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
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
}  // namespace surf_drone
