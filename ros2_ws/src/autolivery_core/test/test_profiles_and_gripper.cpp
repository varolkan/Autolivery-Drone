#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "autolivery_core/gripper.hpp"
#include "autolivery_core/vehicle_profile.hpp"
#include "test_support.hpp"

using namespace autolivery_core;  // NOLINT

namespace
{

bool mentions(const std::vector<std::string> & problems, const std::string & word)
{
  for (const auto & p : problems) {
    if (p.find(word) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(VehicleProfile, TestProfilesAreValid)
{
  EXPECT_TRUE(validate(autolivery_test::quadProfile()).empty());
  EXPECT_TRUE(validate(autolivery_test::planeProfile()).empty());
  EXPECT_TRUE(validate(autolivery_test::vtolProfile()).empty());
}

TEST(VehicleProfile, FixedWingCannotHoverDeliver)
{
  VehicleProfile p = autolivery_test::planeProfile();
  p.dropoff_method = DropoffMethod::Hover;
  EXPECT_TRUE(mentions(validate(p), "air_drop"));
  p = autolivery_test::planeProfile();
  p.min_speed = 0.0;
  EXPECT_TRUE(mentions(validate(p), "stall"));
  p = autolivery_test::planeProfile();
  p.acceptance_radius = 5.0;
  EXPECT_TRUE(mentions(validate(p), "acceptance_radius"));
}

TEST(VehicleProfile, CatchesUnsafeNumbers)
{
  VehicleProfile p = autolivery_test::quadProfile();
  p.planning_clearance = 0.3;
  EXPECT_TRUE(mentions(validate(p), "planning_clearance"));
  p = autolivery_test::quadProfile();
  p.cruise_speed = 20.0;
  EXPECT_TRUE(mentions(validate(p), "max_speed"));
  p = autolivery_test::quadProfile();
  p.max_descent_rate = std::nan("");
  EXPECT_TRUE(mentions(validate(p), "max_descent_rate"));
  p = autolivery_test::quadProfile();
  p.takeoff_altitude = 80.0;
  EXPECT_TRUE(mentions(validate(p), "takeoff_altitude"));
}

TEST(VehicleProfile, ParsingRoundTrips)
{
  for (auto c : {VehicleClass::Multicopter, VehicleClass::FixedWing, VehicleClass::Vtol}) {
    EXPECT_EQ(parseVehicleClass(toString(c)), c);
  }
  for (auto m : {DropoffMethod::Hover, DropoffMethod::Winch, DropoffMethod::Land,
      DropoffMethod::AirDrop})
  {
    EXPECT_EQ(parseDropoffMethod(toString(m)), m);
  }
  EXPECT_EQ(parseVehicleClass("MultiRotor"), VehicleClass::Multicopter);
  EXPECT_THROW(parseVehicleClass("helicopter"), std::invalid_argument);
  EXPECT_THROW(parseDropoffMethod("catapult"), std::invalid_argument);
}

TEST(Gripper, MavlinkGripperUsesDoGripper)
{
  GripperConfig c;
  c.type = GripperType::MavlinkGripper;
  c.instance = 2;
  const auto release = gripperCommands(c, GripperAction::Release);
  ASSERT_EQ(release.size(), 1u);
  EXPECT_EQ(release[0].command, kMavCmdDoGripper);
  EXPECT_EQ(release[0].params[0], 2.0f);
  EXPECT_EQ(release[0].params[1], 0.0f);
  EXPECT_EQ(gripperCommands(c, GripperAction::Grab)[0].params[1], 1.0f);
}

TEST(Gripper, ActuatorGrippersOnlyTouchTheirOwnOutput)
{
  GripperConfig c;
  c.type = GripperType::Servo;
  c.actuator = 3;
  c.release_value = -0.5f;
  const auto cmd = gripperCommands(c, GripperAction::Release);
  ASSERT_EQ(cmd.size(), 1u);
  EXPECT_EQ(cmd[0].command, kMavCmdDoSetActuator);
  for (int i = 0; i < 6; ++i) {
    if (i == 2) {
      EXPECT_EQ(cmd[0].params[i], -0.5f);
    } else {
      EXPECT_TRUE(std::isnan(cmd[0].params[i])) << "output " << i + 1 << " would be driven";
    }
  }
  EXPECT_EQ(cmd[0].params[6], 0.0f);

  c.type = GripperType::Electromagnet;
  EXPECT_EQ(gripperCommands(c, GripperAction::Grab)[0].params[2], 1.0f);
  EXPECT_NE(
    gripperCommands(c, GripperAction::Release)[0].description.find("off"),
    std::string::npos);
}

TEST(Gripper, WinchLowersThenRetractsAndLocks)
{
  GripperConfig c;
  c.type = GripperType::Winch;
  auto cmds = gripperCommands(c, GripperAction::Release);
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].command, kMavCmdDoWinch);
  EXPECT_EQ(cmds[0].params[1], 4.0f);  // WINCH_DELIVER
  c.winch_length = 12.0;
  cmds = gripperCommands(c, GripperAction::Release);
  EXPECT_EQ(cmds[0].params[1], 1.0f);  // relative length control
  EXPECT_EQ(cmds[0].params[2], 12.0f);
  cmds = gripperCommands(c, GripperAction::Grab);
  ASSERT_EQ(cmds.size(), 2u);
  EXPECT_EQ(cmds[0].params[1], 6.0f);  // retract
  EXPECT_EQ(cmds[1].params[1], 3.0f);  // lock
  EXPECT_NEAR(releaseDuration(c, 10.0), c.release_time + 12.0 / c.winch_rate, 1e-9);
}

TEST(Gripper, SimulatedHasNoMavlinkAndBadConfigThrows)
{
  GripperConfig c;
  EXPECT_TRUE(gripperCommands(c, GripperAction::Release).empty());
  c.type = GripperType::Servo;
  c.actuator = 7;
  EXPECT_THROW(gripperCommands(c, GripperAction::Release), std::invalid_argument);
  c.actuator = 1;
  c.release_value = 1.5f;
  EXPECT_THROW(gripperCommands(c, GripperAction::Release), std::invalid_argument);
  EXPECT_EQ(parseGripperType("electromagnet"), GripperType::Electromagnet);
  EXPECT_THROW(parseGripperType("tentacle"), std::invalid_argument);
}
