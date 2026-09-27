// main() for every Autolivery node, with a deadline on shutdown.
//
// Destroying the last node in a process destroys its DDS participant, and Fast DDS 2.14 can
// livelock there when other participants leave at the same moment, which on a launch
// shutdown they all do: DomainParticipant::set_listener(nullptr) spins at 100 % CPU waiting
// for a discovery callback that is starved of the lock it spins on. The node then ignores
// SIGINT until whatever supervises it escalates to SIGKILL (about 1 in 15 dry-run shutdowns).
// Nothing is left to do by then, so once shutdown is under way the process gets
// kShutdownDeadline to finish, and after that exits anyway with the status it would have had.
//
// The clock starts at whichever comes first:
//   - rclcpp's pre-shutdown hook, i.e. SIGINT/SIGTERM, wherever main() is at the time;
//   - ~AutoliveryNode, which runs before the rclcpp::Node underneath is torn down, including
//     when a node's constructor throws on a configuration error.

#ifndef AUTOLIVERY_DRONE__NODE_MAIN_HPP_
#define AUTOLIVERY_DRONE__NODE_MAIN_HPP_

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

namespace autolivery_drone
{

// Inside launch's 5 s SIGINT -> SIGTERM escalation, so a stuck teardown ends as an exit and
// not a kill, and well over the longest callback (the 1 s RRT* budget) so a clean one is never
// cut short.
constexpr std::chrono::milliseconds kShutdownDeadline{3000};

// Start the teardown clock. The process exits with the highest exit_code armed if the clock
// runs out. Thread-safe; later calls only raise the code.
void armShutdownDeadline(int exit_code);

// Base of every node. Arms the deadline (with 1 if an exception is unwinding) before the ROS
// side of the node goes away.
class AutoliveryNode : public rclcpp::Node
{
public:
  using rclcpp::Node::Node;
  ~AutoliveryNode() override;
};

// Enforces the deadline while it lives: exits the process if teardown overruns it.
class ShutdownWatchdog
{
public:
  explicit ShutdownWatchdog(std::string name);
  ~ShutdownWatchdog();
  ShutdownWatchdog(const ShutdownWatchdog &) = delete;
  ShutdownWatchdog & operator=(const ShutdownWatchdog &) = delete;

private:
  rclcpp::PreShutdownCallbackHandle hook_;
  std::thread thread_;
};

template<typename NodeT>
int runNode(int argc, char ** argv, const char * name)
{
  static_assert(
    std::is_base_of_v<AutoliveryNode, NodeT>, "nodes derive from AutoliveryNode");
  rclcpp::init(argc, argv);
  ShutdownWatchdog watchdog(name);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<NodeT>());
  } catch (const std::exception & e) {
    // Configuration errors (bad profile, missing map, unknown depot) end up here. Exit
    // non-zero so launch and systemd see a failed start rather than a quiet node.
    RCLCPP_FATAL(rclcpp::get_logger(name), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__NODE_MAIN_HPP_
