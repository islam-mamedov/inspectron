#include "inspectron_motion_controller/motion_policy.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace {

using inspectron::motion::MotionCommand;
using inspectron::motion::MotionLimits;
using inspectron::motion::PolicyMode;
using inspectron::motion::enforce_motion_policy;

constexpr MotionLimits kLimits{
  0.40,
  0.80,
  0.35,
};

TEST(MotionPolicyTest, ProceedClampsRequestedVelocity)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{1.20, -2.00},
    PolicyMode::Proceed,
    true,
    true,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.40);
  EXPECT_DOUBLE_EQ(result.angular_z, -0.80);
}

TEST(MotionPolicyTest, SlowScalesBoundedVelocity)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Slow,
    true,
    true,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.07);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.14);
}

TEST(MotionPolicyTest, StopAlwaysProducesZeroVelocity)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Stop,
    true,
    true,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

TEST(MotionPolicyTest, InvalidPolicyFailsClosed)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Proceed,
    false,
    true,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

TEST(MotionPolicyTest, StalePolicyFailsClosed)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Proceed,
    true,
    false,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

TEST(MotionPolicyTest, StaleCommandFailsClosed)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Proceed,
    true,
    true,
    false,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

TEST(MotionPolicyTest, NonFiniteCommandFailsClosed)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{
      std::numeric_limits<double>::quiet_NaN(),
      0.40,
    },
    PolicyMode::Proceed,
    true,
    true,
    true,
    kLimits);

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

TEST(MotionPolicyTest, InvalidLimitsFailClosed)
{
  const MotionCommand result = enforce_motion_policy(
    MotionCommand{0.20, 0.40},
    PolicyMode::Proceed,
    true,
    true,
    true,
    MotionLimits{0.0, 0.80, 0.35});

  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

}  // namespace
