// Runs a slow call without starving a periodic duty.
//
// mission_node plans routes from inside its control tick. On the ground that costs nothing.
// In flight (an abort, an air-drop go-around) a route across a city can take longer than PX4
// tolerates without setpoints before it declares the offboard link lost and fails safe out
// of OFFBOARD -- in the middle of the very manoeuvre that was meant to bring the vehicle home.
// So the slow call runs on another thread while this one keeps doing the periodic duty.

#ifndef AUTOLIVERY_DRONE__KEEP_ALIVE_HPP_
#define AUTOLIVERY_DRONE__KEEP_ALIVE_HPP_

#include <chrono>
#include <functional>
#include <future>

namespace autolivery_drone
{

// Calls `work` on a helper thread and `keep_alive` on this one every `period` until the work
// is done, then returns its result (or rethrows what it threw).
//
// `max_keep_alive` bounds the favour: work that never comes back must not keep an unattended
// vehicle looking healthy to its autopilot. Past it the duty is dropped (the wait goes on),
// so the autopilot's own failsafe takes over exactly as it would had this process died.
template<typename Result>
Result runWithKeepAlive(
  const std::function<Result()> & work, const std::function<void()> & keep_alive,
  std::chrono::duration<double> period, std::chrono::duration<double> max_keep_alive)
{
  auto future = std::async(std::launch::async, work);
  const auto start = std::chrono::steady_clock::now();
  while (future.wait_for(period) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - start <= max_keep_alive) {
      keep_alive();
    }
  }
  return future.get();
}

}  // namespace autolivery_drone

#endif  // AUTOLIVERY_DRONE__KEEP_ALIVE_HPP_
