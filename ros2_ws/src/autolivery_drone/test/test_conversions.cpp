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

TEST(Conversions, WindFromTheCompassIsAirMovingTheOtherWay)
{
  // A westerly comes from 270 and moves the air east; a northerly moves it south.
  Vec3 w = windFromMeteorological(10.0, 270.0);
  EXPECT_NEAR(w.x, 10.0, 1e-9);
  EXPECT_NEAR(w.y, 0.0, 1e-9);
  w = windFromMeteorological(10.0, 0.0);
  EXPECT_NEAR(w.x, 0.0, 1e-9);
  EXPECT_NEAR(w.y, -10.0, 1e-9);
  w = windFromMeteorological(10.0, 90.0);
  EXPECT_NEAR(w.x, -10.0, 1e-9);
  w = windFromMeteorological(10.0, 180.0);
  EXPECT_NEAR(w.y, 10.0, 1e-9);
  for (double from : {0.0, 45.0, 123.0, 270.0, 359.0}) {
    EXPECT_NEAR(windDirection(windFromMeteorological(7.0, from)), from, 1e-6);
  }
}

TEST(Conversions, WeatherReportKeepsWhatWasReportedAndNothingElse)
{
  const float not_reported = std::numeric_limits<float>::quiet_NaN();
  autolivery_interfaces::msg::Weather m;
  m.wind_speed = 6.0f;
  m.wind_direction = 315.0f;  // a north-westerly: towards the south-east
  m.wind_gust = 9.5f;
  m.precipitation = 0.4f;
  m.temperature = -3.5f;
  auto w = weatherFromMsg(m);
  ASSERT_TRUE(w.wind.has_value());
  EXPECT_NEAR(w.wind->horizontalNorm(), 6.0, 1e-5);
  EXPECT_NEAR(w.wind->x, 6.0 / std::sqrt(2.0), 1e-5);
  EXPECT_NEAR(w.wind->y, -6.0 / std::sqrt(2.0), 1e-5);
  EXPECT_FLOAT_EQ(static_cast<float>(w.gust), 9.5f);
  EXPECT_FLOAT_EQ(static_cast<float>(w.precipitation), 0.4f);
  EXPECT_FLOAT_EQ(static_cast<float>(w.temperature), -3.5f);

  // A report with nothing in it says nothing, rather than "calm, dry and 0 C".
  m.wind_speed = m.wind_direction = m.wind_gust = m.precipitation = m.temperature = not_reported;
  w = weatherFromMsg(m);
  EXPECT_FALSE(w.wind.has_value());
  EXPECT_LT(w.gust, 0.0);
  EXPECT_LT(w.precipitation, 0.0);
  EXPECT_TRUE(std::isnan(w.temperature));

  // "Light and variable": a speed with no direction still counts as a speed.
  m.wind_speed = 1.5f;
  w = weatherFromMsg(m);
  ASSERT_TRUE(w.wind.has_value());
  EXPECT_NEAR(w.wind->horizontalNorm(), 1.5, 1e-6);

  // A negative reading is a broken sensor, not a reading.
  m.wind_speed = -1.0f;
  m.wind_gust = -5.0f;
  m.precipitation = -0.1f;
  w = weatherFromMsg(m);
  EXPECT_FALSE(w.wind.has_value());
  EXPECT_LT(w.gust, 0.0);
  EXPECT_LT(w.precipitation, 0.0);
}

TEST(Conversions, YawQuaternion)
{
  for (double yaw : {0.0, 0.5, -2.0, 3.0}) {
    EXPECT_NEAR(yawFromQuaternion(quaternionFromYaw(yaw)), yaw, 1e-12);
  }
}
