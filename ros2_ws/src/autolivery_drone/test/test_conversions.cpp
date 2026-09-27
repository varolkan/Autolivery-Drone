#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "autolivery_drone/conversions.hpp"

using autolivery_core::Setpoint;
using autolivery_core::Vec3;
using mavros_msgs::msg::PositionTarget;
using namespace autolivery_drone;  // NOLINT

TEST(Conversions, VelocitySetpointMasksPositionAndKeepsYaw)
{
  Setpoint sp;
  sp.kind = Setpoint::Kind::Velocity;
  sp.velocity = {3.0, -1.0, 0.5};
  sp.yaw = 1.2;
  const auto pt = toPositionTarget(sp, {100.0, 50.0, 0.0});
  EXPECT_EQ(pt.coordinate_frame, PositionTarget::FRAME_LOCAL_NED);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_PX);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_PZ);
  EXPECT_FALSE(pt.type_mask & PositionTarget::IGNORE_VX);
  EXPECT_FALSE(pt.type_mask & PositionTarget::IGNORE_YAW);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_YAW_RATE);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_AFX);
  EXPECT_DOUBLE_EQ(pt.velocity.x, 3.0);  // velocities do not depend on the frame origin
  EXPECT_FLOAT_EQ(pt.yaw, 1.2f);
}

TEST(Conversions, PositionSetpointIsShiftedIntoTheLocalFrame)
{
  Setpoint sp;
  sp.kind = Setpoint::Kind::Position;
  sp.position = {110.0, 60.0, 40.0};
  const auto pt = toPositionTarget(sp, {100.0, 50.0, 2.0});
  EXPECT_DOUBLE_EQ(pt.position.x, 10.0);
  EXPECT_DOUBLE_EQ(pt.position.y, 10.0);
  EXPECT_DOUBLE_EQ(pt.position.z, 38.0);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_VX);
  EXPECT_FALSE(pt.type_mask & PositionTarget::IGNORE_PX);
  EXPECT_TRUE(pt.type_mask & PositionTarget::IGNORE_YAW);  // NaN yaw = hold heading

  const Setpoint back = fromPositionTarget(pt, {100.0, 50.0, 2.0});
  EXPECT_EQ(back.kind, Setpoint::Kind::Position);
  EXPECT_DOUBLE_EQ(back.position.x, 110.0);
  EXPECT_TRUE(std::isnan(back.yaw));
}

TEST(Conversions, MavrosEnumsRoundTrip)
{
  using autolivery_core::LandedState;
  using autolivery_core::VtolState;
  for (auto s : {LandedState::OnGround, LandedState::InAir, LandedState::Takeoff,
      LandedState::Landing, LandedState::Unknown})
  {
    EXPECT_EQ(landedFromMavros(landedToMavros(s)), s);
  }
  for (auto s : {VtolState::Mc, VtolState::Fw, VtolState::TransitionToFw,
      VtolState::TransitionToMc, VtolState::Undefined})
  {
    EXPECT_EQ(vtolFromMavros(vtolToMavros(s)), s);
  }
}

TEST(Conversions, YawQuaternion)
{
  for (double yaw : {0.0, 0.5, -2.0, 3.0}) {
    EXPECT_NEAR(yawFromQuaternion(quaternionFromYaw(yaw)), yaw, 1e-12);
  }
}
