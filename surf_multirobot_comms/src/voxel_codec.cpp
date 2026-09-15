#include "surf_multirobot_comms/voxel_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <tuple>
#include <utility>

#include <zstd.h>

namespace surf::comms
{
namespace
{

struct Record
{
  int32_t x;
  int32_t y;
  int32_t z;
  uint8_t state;
  uint64_t observation_time_ns;
  uint8_t ray_flag{0};
  geometry_msgs::msg::Point ray_origin, ray_endpoint;
};

void append_double(double value, std::vector<uint8_t> & output)
{
  uint64_t bits; std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 8; ++i) output.push_back(static_cast<uint8_t>(bits >> (i * 8)));
}

bool read_double(const std::vector<uint8_t> & input, std::size_t & offset, double & value)
{
  if (input.size() - offset < 8) return false;
  uint64_t bits = 0;
  for (int i = 0; i < 8; ++i) bits |= static_cast<uint64_t>(input[offset++]) << (i * 8);
  std::memcpy(&value, &bits, sizeof(value));
  return std::isfinite(value);
}

void append_varuint(uint64_t value, std::vector<uint8_t> & output)
{
  while (value >= 0x80U) {
    output.push_back(static_cast<uint8_t>(value) | 0x80U);
    value >>= 7U;
  }
  output.push_back(static_cast<uint8_t>(value));
}

uint64_t zigzag(int64_t value)
{
  return (static_cast<uint64_t>(value) << 1U) ^
         static_cast<uint64_t>(value >> 63U);
}

int64_t unzigzag(uint64_t value)
{
  return static_cast<int64_t>((value >> 1U) ^
    static_cast<uint64_t>(-static_cast<int64_t>(value & 1U)));
}

bool read_varuint(
  const std::vector<uint8_t> & input, std::size_t & offset, uint64_t & value)
{
  value = 0U;
  for (unsigned shift = 0U; shift < 64U; shift += 7U) {
    if (offset >= input.size()) {
      return false;
    }
    const uint8_t byte = input[offset++];
    value |= static_cast<uint64_t>(byte & 0x7fU) << shift;
    if ((byte & 0x80U) == 0U) {
      return true;
    }
  }
  return false;
}

void copy_metadata(
  const surf_multirobot_msgs::msg::VoxelDelta & input,
  surf_multirobot_msgs::msg::CompressedVoxelDelta & output)
{
  output.header = input.header;
  output.source_id = input.source_id;
  output.map_epoch = input.map_epoch;
  output.version = input.version;
  output.base_version = input.base_version;
  output.operating_mode = input.operating_mode;
  output.full_refresh = input.full_refresh;
  output.chunk_index = input.chunk_index;
  output.chunk_count = input.chunk_count;
  output.resolution = input.resolution;
  output.sensor_origin = input.sensor_origin;
}

void copy_metadata(
  const surf_multirobot_msgs::msg::CompressedVoxelDelta & input,
  surf_multirobot_msgs::msg::VoxelDelta & output)
{
  output.header = input.header;
  output.source_id = input.source_id;
  output.map_epoch = input.map_epoch;
  output.version = input.version;
  output.base_version = input.base_version;
  output.operating_mode = input.operating_mode;
  output.full_refresh = input.full_refresh;
  output.chunk_index = input.chunk_index;
  output.chunk_count = input.chunk_count;
  output.resolution = input.resolution;
  output.sensor_origin = input.sensor_origin;
}

}  // namespace

CodecResult encode_delta(
  const surf_multirobot_msgs::msg::VoxelDelta & delta,
  surf_multirobot_msgs::msg::CompressedVoxelDelta & output,
  int compression_level)
{
  const std::size_t count = delta.x.size();
  if (delta.y.size() != count || delta.z.size() != count || delta.state.size() != count ||
    delta.observation_time_ns.size() != count)
  {
    return {false, "voxel arrays have different lengths"};
  }
  if (count > std::numeric_limits<uint32_t>::max()) {
    return {false, "too many voxel records"};
  }
  const bool has_ray_arrays = !delta.ray_flags.empty();
  if (!has_ray_arrays && (!delta.ray_origins.empty() || !delta.ray_endpoints.empty()))
    return {false, "ray metadata without flags"};
  if (has_ray_arrays && (delta.ray_flags.size() != count || delta.ray_origins.size() != count ||
    delta.ray_endpoints.size() != count)) return {false, "ray arrays have different lengths"};
  const bool rays = has_ray_arrays &&
    std::any_of(delta.ray_flags.begin(), delta.ray_flags.end(), [](uint8_t flag) {return flag != 0;});

  std::vector<Record> records;
  records.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    Record record{
      delta.x[index], delta.y[index], delta.z[index], delta.state[index],
      delta.observation_time_ns[index], 0,
      geometry_msgs::msg::Point(), geometry_msgs::msg::Point()};
    if (has_ray_arrays) {
      record.ray_flag = delta.ray_flags[index];
      record.ray_origin = delta.ray_origins[index];
      record.ray_endpoint = delta.ray_endpoints[index];
      if (record.ray_flag > 2 || (record.ray_flag == 1 && record.state !=
        surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) ||
        (record.ray_flag == 2 && record.state != surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC &&
        record.state != surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC))
        return {false, "invalid ray flag"};
    }
    records.push_back(record);
  }
  std::sort(records.begin(), records.end(), [](const Record & lhs, const Record & rhs) {
    return std::tie(lhs.x, lhs.y, lhs.z, lhs.state, lhs.observation_time_ns) <
           std::tie(rhs.x, rhs.y, rhs.z, rhs.state, rhs.observation_time_ns);
  });

  std::vector<uint8_t> plain;
  plain.reserve(8U + count * 13U);
  plain.insert(plain.end(), {'S', 'V', 'D', static_cast<uint8_t>(rays ? '3' : '2')});
  append_varuint(count, plain);
  int64_t previous_x = 0;
  int64_t previous_y = 0;
  int64_t previous_z = 0;
  for (const auto & record : records) {
    append_varuint(zigzag(static_cast<int64_t>(record.x) - previous_x), plain);
    append_varuint(zigzag(static_cast<int64_t>(record.y) - previous_y), plain);
    append_varuint(zigzag(static_cast<int64_t>(record.z) - previous_z), plain);
    plain.push_back(record.state);
    append_varuint(record.observation_time_ns, plain);
    if (rays) {
      plain.push_back(record.ray_flag);
      if (record.ray_flag) {
        const std::array<double, 6> geometry{record.ray_origin.x, record.ray_origin.y, record.ray_origin.z,
          record.ray_endpoint.x, record.ray_endpoint.y, record.ray_endpoint.z};
        for (std::size_t i = 0; i < (record.ray_flag == 2 ? 3U : 6U); ++i) {
          const double value = geometry[i];
          if (!std::isfinite(value))
            return {false, "invalid ray geometry"};
          append_double(value, plain);
        }
      }
    }
    previous_x = record.x;
    previous_y = record.y;
    previous_z = record.z;
  }

  copy_metadata(delta, output);
  output.voxel_count = static_cast<uint32_t>(count);
  output.uncompressed_bytes = static_cast<uint32_t>(plain.size());

  std::vector<uint8_t> compressed(ZSTD_compressBound(plain.size()));

  ZSTD_CCtx * context = ZSTD_createCCtx();
  if (!context) {
    return {false, "could not allocate zstd compression context"};
  }
  ZSTD_CCtx_setParameter(context, ZSTD_c_compressionLevel, compression_level);
  ZSTD_CCtx_setParameter(context, ZSTD_c_checksumFlag, 1);
  const std::size_t compressed_size = ZSTD_compress2(
    context, compressed.data(), compressed.size(), plain.data(), plain.size());
  ZSTD_freeCCtx(context);
  if (ZSTD_isError(compressed_size)) {
    return {false, ZSTD_getErrorName(compressed_size)};
  }
  compressed.resize(compressed_size);

  // A zstd frame can be larger than a very small sparse delta. Keep the same
  // deterministic SVD2 representation without the frame in that case.
  if (compressed.size() < plain.size()) {
    output.codec = rays ? "zstd-svd3" : "zstd-svd2";
    output.payload = std::move(compressed);
  } else {
    output.codec = rays ? "raw-svd3" : "raw-svd2";
    output.payload = std::move(plain);
  }
  return {true, {}};
}

CodecResult decode_delta(
  const surf_multirobot_msgs::msg::CompressedVoxelDelta & input,
  surf_multirobot_msgs::msg::VoxelDelta & delta,
  std::size_t maximum_uncompressed_bytes)
{
  if (input.uncompressed_bytes > maximum_uncompressed_bytes) {
    return {false, "uncompressed payload exceeds configured limit"};
  }

  std::vector<uint8_t> plain;
  const bool has_voxel_timestamps =
    input.codec == "raw-svd2" || input.codec == "zstd-svd2" ||
    input.codec == "raw-svd3" || input.codec == "zstd-svd3";
  const bool rays = input.codec == "raw-svd3" || input.codec == "zstd-svd3";
  if (input.codec == "raw-svd1" || input.codec == "raw-svd2" || input.codec == "raw-svd3") {
    if (input.payload.size() != input.uncompressed_bytes) {
      return {false, "raw payload size does not match metadata"};
    }
    plain = input.payload;
  } else if (input.codec == "zstd-svd1" || input.codec == "zstd-svd2" || input.codec == "zstd-svd3") {
    plain.resize(input.uncompressed_bytes);
    const std::size_t decoded_size = ZSTD_decompress(
      plain.data(), plain.size(), input.payload.data(), input.payload.size());
    if (ZSTD_isError(decoded_size)) {
      return {false, ZSTD_getErrorName(decoded_size)};
    }
    if (decoded_size != plain.size()) {
      return {false, "decoded payload size does not match metadata"};
    }
  } else {
    return {false, "unsupported codec: " + input.codec};
  }

  const std::array<uint8_t, 4> magic{
    'S', 'V', 'D', static_cast<uint8_t>(rays ? '3' : (has_voxel_timestamps ? '2' : '1'))};
  if (plain.size() < 5U || !std::equal(plain.begin(), plain.begin() + 4, magic.begin()))
  {
    return {false, "invalid sparse voxel delta payload"};
  }

  std::size_t offset = 4U;
  uint64_t count = 0U;
  if (!read_varuint(plain, offset, count) || count != input.voxel_count ||
    count > std::numeric_limits<uint32_t>::max())
  {
    return {false, "invalid voxel count"};
  }

  copy_metadata(input, delta);
  delta.x.clear();
  delta.y.clear();
  delta.z.clear();
  delta.state.clear();
  delta.observation_time_ns.clear();
  delta.ray_flags.clear(); delta.ray_origins.clear(); delta.ray_endpoints.clear();
  delta.x.reserve(count);
  delta.y.reserve(count);
  delta.z.reserve(count);
  delta.state.reserve(count);
  delta.observation_time_ns.reserve(count);
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  for (uint64_t index = 0; index < count; ++index) {
    uint64_t dx = 0U;
    uint64_t dy = 0U;
    uint64_t dz = 0U;
    if (!read_varuint(plain, offset, dx) || !read_varuint(plain, offset, dy) ||
      !read_varuint(plain, offset, dz) || offset >= plain.size())
    {
      return {false, "truncated voxel record"};
    }
    x += unzigzag(dx);
    y += unzigzag(dy);
    z += unzigzag(dz);
    if (x < std::numeric_limits<int32_t>::min() || x > std::numeric_limits<int32_t>::max() ||
      y < std::numeric_limits<int32_t>::min() || y > std::numeric_limits<int32_t>::max() ||
      z < std::numeric_limits<int32_t>::min() || z > std::numeric_limits<int32_t>::max())
    {
      return {false, "voxel coordinate overflow"};
    }
    delta.x.push_back(static_cast<int32_t>(x));
    delta.y.push_back(static_cast<int32_t>(y));
    delta.z.push_back(static_cast<int32_t>(z));
    delta.state.push_back(plain[offset++]);
    uint64_t observation_time_ns =
      static_cast<uint64_t>(std::max(input.header.stamp.sec, 0)) * 1000000000ULL +
      static_cast<uint64_t>(input.header.stamp.nanosec);
    if (has_voxel_timestamps && !read_varuint(plain, offset, observation_time_ns)) {
      return {false, "truncated voxel timestamp"};
    }
    delta.observation_time_ns.push_back(observation_time_ns);
    if (rays) {
      if (offset >= plain.size()) return {false, "truncated ray flag"};
      const uint8_t flag = plain[offset++];
      if (flag > 2 || (flag == 1 && delta.state.back() !=
        surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) ||
        (flag == 2 && delta.state.back() != surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC &&
        delta.state.back() != surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC))
        return {false, "invalid ray flag"};
      geometry_msgs::msg::Point origin, endpoint;
      if (flag) {
        double values[6]{};
        for (std::size_t i = 0; i < (flag == 2 ? 3U : 6U); ++i) if (!read_double(plain, offset, values[i]))
          return {false, "invalid ray geometry"};
        origin.x = values[0]; origin.y = values[1]; origin.z = values[2];
        endpoint.x = values[3]; endpoint.y = values[4]; endpoint.z = values[5];
      }
      delta.ray_flags.push_back(flag);
      delta.ray_origins.push_back(origin);
      delta.ray_endpoints.push_back(endpoint);
    }
  }
  if (offset != plain.size()) {
    return {false, "trailing bytes in voxel payload"};
  }
  return {true, {}};
}

}  // namespace surf::comms
