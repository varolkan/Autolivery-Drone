// runWithKeepAlive: the periodic duty goes on while slow work runs elsewhere, and stops being
// done for work that never comes back.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "autolivery_drone/keep_alive.hpp"

using autolivery_drone::runWithKeepAlive;
using std::chrono::milliseconds;

namespace
{

std::function<int()> slowWork(milliseconds duration, int result)
{
  return [duration, result]() {
           std::this_thread::sleep_for(duration);
           return result;
         };
}

}  // namespace

TEST(KeepAlive, DutyIsDoneAtItsPeriodWhileTheWorkRuns)
{
  int beats = 0;
  const std::thread::id caller = std::this_thread::get_id();
  bool on_caller_thread = true;
  const int result = runWithKeepAlive<int>(
    slowWork(milliseconds(400), 42), [&]() {
      ++beats;
      on_caller_thread = on_caller_thread && std::this_thread::get_id() == caller;
    },
    milliseconds(50), milliseconds(5000));
  EXPECT_EQ(result, 42);
  // 400 ms at 50 ms: about 7, never fewer than a 2 Hz-safe handful even on a loaded machine.
  EXPECT_GE(beats, 4);
  EXPECT_LE(beats, 8);
  EXPECT_TRUE(on_caller_thread);  // the duty (publishing) stays on the thread that owns it
}

TEST(KeepAlive, WorkRunsOffTheCallingThread)
{
  const std::thread::id caller = std::this_thread::get_id();
  std::thread::id worker;
  runWithKeepAlive<int>(
    [&]() {
      worker = std::this_thread::get_id();
      return 0;
    }, []() {}, milliseconds(50), milliseconds(1000));
  EXPECT_NE(worker, caller);
}

TEST(KeepAlive, FastWorkCostsNoExtraBeats)
{
  int beats = 0;
  EXPECT_EQ(
    runWithKeepAlive<int>(
      []() {return 7;}, [&]() {++beats;}, milliseconds(200), milliseconds(1000)), 7);
  EXPECT_EQ(beats, 0);
}

TEST(KeepAlive, StopsVouchingForWorkThatOverstays)
{
  // 600 ms of work but only 150 ms of cover: the duty is dropped (so the autopilot notices)
  // while the call still waits for the result.
  int beats = 0;
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(
    runWithKeepAlive<int>(
      slowWork(milliseconds(600), 1), [&]() {++beats;}, milliseconds(50), milliseconds(150)), 1);
  const auto took = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(beats, 1);
  EXPECT_LE(beats, 4);
  EXPECT_GE(took, milliseconds(590));
}

TEST(KeepAlive, WhatTheWorkThrowsReachesTheCaller)
{
  EXPECT_THROW(
    runWithKeepAlive<int>(
      []() -> int {throw std::runtime_error("planner fell over");}, []() {},
      milliseconds(50), milliseconds(1000)),
    std::runtime_error);
}
