#pragma once

#include <cstdint>

namespace inspectron::motion {

enum class PolicyMode : std::uint8_t {
  Stop = 0,
  Proceed = 1,
  Slow = 2,
};

struct MotionCommand {
  double linear_x{0.0};
  double angular_z{0.0};
};

struct MotionLimits {
  double max_linear_speed{0.40};
  double max_angular_speed{0.80};
  double slow_scale{0.35};
};

[[nodiscard]] MotionCommand enforce_motion_policy(
  const MotionCommand & requested,
  PolicyMode mode,
  bool policy_valid,
  bool policy_fresh,
  bool command_fresh,
  const MotionLimits & limits) noexcept;

}  // namespace inspectron::motion
