#include "autolivery_drone/node_main.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <utility>

namespace autolivery_drone
{

namespace
{

struct Deadline
{
  std::mutex mutex;
  std::condition_variable cv;
  bool armed{false};
  bool stopped{false};  // main() got through teardown in time
  int exit_code{0};
  std::chrono::steady_clock::time_point expires;
};

Deadline & deadline()
{
  static Deadline d;
  return d;
}

}  // namespace

void armShutdownDeadline(int exit_code)
{
  Deadline & d = deadline();
  const std::lock_guard<std::mutex> lock(d.mutex);
  d.exit_code = std::max(d.exit_code, exit_code);
  if (!d.armed) {
    d.armed = true;
    d.expires = std::chrono::steady_clock::now() + kShutdownDeadline;
    d.cv.notify_all();
  }
}

AutoliveryNode::~AutoliveryNode()
{
  armShutdownDeadline(std::uncaught_exceptions() > 0 ? 1 : 0);
}

ShutdownWatchdog::ShutdownWatchdog(std::string name)
{
  {
    Deadline & d = deadline();
    const std::lock_guard<std::mutex> lock(d.mutex);
    d.armed = false;
    d.stopped = false;
    d.exit_code = 0;
  }
  hook_ = rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [] {armShutdownDeadline(0);});
  thread_ = std::thread(
    [name = std::move(name)] {
      Deadline & d = deadline();
      std::unique_lock<std::mutex> lock(d.mutex);
      d.cv.wait(lock, [&d] {return d.armed || d.stopped;});
      if (d.cv.wait_until(lock, d.expires, [&d] {return d.stopped;})) {
        return;
      }
      // Straight to stderr: rclcpp logging may already be finalised, and the main thread is
      // stuck somewhere we cannot safely wait on.
      std::fprintf(
        stderr, "[%s] still tearing down %.1f s after shutdown began (DDS participant "
        "teardown, see node_main.hpp); exiting with %d without finishing it\n", name.c_str(),
        std::chrono::duration<double>(kShutdownDeadline).count(), d.exit_code);
      std::fflush(stderr);
      std::_Exit(d.exit_code);
    });
}

ShutdownWatchdog::~ShutdownWatchdog()
{
  rclcpp::contexts::get_global_default_context()->remove_pre_shutdown_callback(hook_);
  Deadline & d = deadline();
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    d.stopped = true;
  }
  d.cv.notify_all();
  thread_.join();
}

}  // namespace autolivery_drone
