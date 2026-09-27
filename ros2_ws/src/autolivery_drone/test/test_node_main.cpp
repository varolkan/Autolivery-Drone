// The shutdown deadline in node_main.hpp. The Fast DDS teardown livelock it guards against is
// a race that cannot be triggered on demand, so each test stands in for it with a teardown
// that never returns, in a child process (gtest death test), and checks how the process ends.

#include <gtest/gtest.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>

#include "autolivery_drone/node_main.hpp"

using autolivery_drone::AutoliveryNode;
using autolivery_drone::runNode;
using autolivery_drone::ShutdownWatchdog;

namespace
{

// Stands in for the livelocked teardown. Gives up long after the deadline, so a missing
// deadline fails the test with exit code 99 instead of hanging it.
[[noreturn]] void hangLikeDdsTeardown()
{
  std::this_thread::sleep_for(3 * autolivery_drone::kShutdownDeadline);
  std::_Exit(99);
}

struct MisconfiguredNode : AutoliveryNode
{
  MisconfiguredNode()
  : AutoliveryNode("misconfigured")
  {
    throw std::runtime_error("unknown depot 'nowhere'");
  }
};

struct ShortLivedNode : AutoliveryNode
{
  ShortLivedNode()
  : AutoliveryNode("short_lived")
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [] {rclcpp::shutdown();});
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

const char kOverran[] = "still tearing down 3.0 s after shutdown began";

class ShutdownDeadline : public ::testing::Test
{
protected:
  void SetUp() override {GTEST_FLAG_SET(death_test_style, "threadsafe");}
};

}  // namespace

TEST_F(ShutdownDeadline, SigintThenStuckTeardownStillExitsCleanly)
{
  EXPECT_EXIT(
  {
    rclcpp::init(0, nullptr);
    ShutdownWatchdog watchdog("test");
    std::raise(SIGINT);
    hangLikeDdsTeardown();
  }, ::testing::ExitedWithCode(0), kOverran);
}

TEST_F(ShutdownDeadline, SigtermThenStuckTeardownStillExitsCleanly)
{
  EXPECT_EXIT(
  {
    rclcpp::init(0, nullptr);
    ShutdownWatchdog watchdog("test");
    std::raise(SIGTERM);
    hangLikeDdsTeardown();
  }, ::testing::ExitedWithCode(0), kOverran);
}

TEST_F(ShutdownDeadline, ConfigErrorThenStuckTeardownStillExitsWithFailure)
{
  // Unwinding a throwing constructor tears the node down before main() can catch anything;
  // the node itself has to start the clock, and with a failure status.
  EXPECT_EXIT(
  {
    rclcpp::init(0, nullptr);
    ShutdownWatchdog watchdog("test");
    try {
      MisconfiguredNode node;
    } catch (const std::exception &) {
      hangLikeDdsTeardown();
    }
  }, ::testing::ExitedWithCode(1), kOverran);
}

TEST_F(ShutdownDeadline, NodeDestroyedAfterSpinStartsTheClock)
{
  EXPECT_EXIT(
  {
    rclcpp::init(0, nullptr);
    ShutdownWatchdog watchdog("test");
    {
      AutoliveryNode node("finished");
    }
    hangLikeDdsTeardown();
  }, ::testing::ExitedWithCode(0), kOverran);
}

TEST_F(ShutdownDeadline, NothingHappensWhileTheNodeRuns)
{
  // No shutdown requested: the watchdog must never end a running node.
  EXPECT_EXIT(
  {
    rclcpp::init(0, nullptr);
    ShutdownWatchdog watchdog("test");
    auto node = std::make_shared<AutoliveryNode>("running");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::_Exit(7);
  }, ::testing::ExitedWithCode(7), "");
}

TEST_F(ShutdownDeadline, CleanShutdownReturnsFromMain)
{
  // runNode returning (42 is only reachable that way) rather than the watchdog exiting.
  EXPECT_EXIT(
    std::exit(runNode<ShortLivedNode>(0, nullptr, "short_lived") == 0 ? 42 : 1),
    ::testing::ExitedWithCode(42), "");
}

TEST_F(ShutdownDeadline, ConfigErrorReturnsFailureFromMain)
{
  EXPECT_EXIT(
    std::exit(runNode<MisconfiguredNode>(0, nullptr, "misconfigured") == 1 ? 43 : 1),
    ::testing::ExitedWithCode(43), "unknown depot 'nowhere'");
}
