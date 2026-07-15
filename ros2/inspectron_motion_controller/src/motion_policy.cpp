#include "inspectron_motion_controller/motion_policy.hpp"

#include <algorithm>
#include <cmath>

namespace inspectron::motion {
namespace {

[[nodiscard]] bool limits_are_valid(const MotionLimits & limits) noexcept
{
  return std::isfinite(limits.max_linear_speed) &&
         std::isfinite(limits.max_angular_speed) &&
         std::isfinite(limits.slow_scale) &&
         limits.max_linear_speed > 0.0 &&
         limits.max_angular_speed > 0.0 &&
         limits.slow_scale >= 0.0 &&
         limits.slow_scale <= 1.0;
}

[[nodiscard]] MotionCommand stop_command() noexcept
{
  return {};
}

}  // namespace

MotionCommand enforce_motion_policy(
  const MotionCommand & requested,
  const PolicyMode mode,
  const bool policy_valid,
  const bool policy_fresh,
  const bool command_fresh,
  const MotionLimits & limits) noexcept
{
  if (!policy_valid || !policy_fresh || !command_fresh) {
    return stop_command();
  }

  if (mode == PolicyMode::Stop) {
    return stop_command();
  }

  if (!limits_are_valid(limits)) {
    return stop_command();
  }

  if (!std::isfinite(requested.linear_x) ||
      !std::isfinite(requested.angular_z)) {
    return stop_command();
  }

  MotionCommand output{
    std::clamp(
      requested.linear_x,
      -limits.max_linear_speed,
      limits.max_linear_speed),
    std::clamp(
      requested.angular_z,
      -limits.max_angular_speed,
      limits.max_angular_speed),
  };

  if (mode == PolicyMode::Slow) {
    output.linear_x *= limits.slow_scale;
    output.angular_z *= limits.slow_scale;
  }

  return output;
}

}  // namespace inspectron::motion
