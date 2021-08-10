// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <memory>

#include "haste/aux/aux.hpp"
#include "haste/tracking.hpp"
#include "haste/types/event_type.hpp"
#include "haste/types/camera.hpp"
#include "haste/io/rpg_dataset.hpp"

namespace haste {
namespace app {

using Tracker = haste::HypothesisPatchTracker;
using TrackerPtr = std::shared_ptr<Tracker>;

using Event = haste::TEvent<Tracker::Scalar>;
using Camera = haste::PinholeRadTanCamera<Tracker::Scalar>;

struct TrackerState {
  using Scalar = Tracker::Scalar;
  using ID = uint64_t;
  Scalar t, x, y, theta;
  ID id;
};

/// Create an instance of tracker type specified and return its shared pointer from base class.
auto createTracker(const std::string& tracker_type, const TrackerState& seed) -> TrackerPtr {
  LOG(INFO) << "Selected tracker type: " << tracker_type;
  if (tracker_type == "correlation") {
    return std::make_shared<haste::CorrelationTracker>(seed.t, seed.x, seed.y, seed.theta);
  } else if (tracker_type == "haste_correlation") {
    return std::make_shared<haste::HasteCorrelationTracker>(seed.t, seed.x, seed.y, seed.theta);
  } else if (tracker_type == "haste_correlation_star") {
    return std::make_shared<haste::HasteCorrelationStarTracker>(seed.t, seed.x, seed.y, seed.theta);
  } else if (tracker_type == "haste_difference") {
    return std::make_shared<haste::HasteDifferenceTracker>(seed.t, seed.x, seed.y, seed.theta);
  } else if (tracker_type == "haste_difference_star") {
    return std::make_shared<haste::HasteDifferenceStarTracker>(seed.t, seed.x, seed.y, seed.theta);
  } else {
    LOG(FATAL) << "Selected tracker type is not recognized.";
    return nullptr;
  }
}

// 下边这两个是在输出时用的,平时基本用不上
/// Read the current state of a tracker and take a provided id to generate a tracker State.
auto composeTrackerState(const TrackerState::ID& tracker_id, const Tracker& tracker) -> TrackerState {
  return {.t = tracker.t(), .x = tracker.x(), .y = tracker.y(), .theta = tracker.theta(), .id = tracker_id};
};
/// Generate a tracker state joining a provided ID and a tracker and append it to a vector of tracker states.
auto appendTrackerState(const TrackerState::ID& tracker_id, const Tracker& tracker, std::vector<TrackerState>& states)
    -> void {
  states.push_back(composeTrackerState(tracker_id, tracker));
}

// 这个str参数就是seed
/// Convert string ("t,x,y,theta,id") to a tracker state.
auto getTrackerStateFromString(const std::string& str) -> TrackerState {
  constexpr auto delimiter = ',';
  auto seed_str = splitString(str, delimiter);
  CHECK(seed_str.size() == 4 || seed_str.size() == 5)
      << "Could not parse tracker state from string: \"" << str << "\". Incorrect number of tokens delimited by \'"
      << delimiter << "\'." << seed_str.size() << " number of tokens, instead of 4 (excluding ID) or 5 (including ID).";

  auto t = (TrackerState::Scalar) std::stod(seed_str[0]);
  auto x = (TrackerState::Scalar) std::stod(seed_str[1]);
  auto y = (TrackerState::Scalar) std::stod(seed_str[2]);
  auto theta = (TrackerState::Scalar) std::stod(seed_str[3]);

  if (seed_str.size() == 4) {
    return TrackerState{.t = t, .x = x, .y = y, .theta = theta};
  } else if (seed_str.size() == 5) {
    auto id = (TrackerState::ID) std::stoul(seed_str[4]);
    return TrackerState{.t = t, .x = x, .y = y, .theta = theta, .id = id};
  } else {
    LOG(FATAL) << "Unexpected number of tokens.";// It should never happen due to the previous check.
  }
}

/// Load tracker states from file.
auto getTrackerStatesFromFile(const std::string& file_path) -> std::vector<TrackerState> {
  LOG(INFO) << "Loading tracker states (Format: t,x,y,theta,id) from file " << file_path << " ...";
  std::ifstream file(file_path);
  CHECK(!file.fail()) << "Error opening file: " << file_path;

  std::vector<TrackerState> states;
  // Loop over lines parsing them as states.
  std::string str_line;
  while (file >> str_line) {
    states.push_back(getTrackerStateFromString(str_line));
  }
  file.close();
  LOG(INFO) << "Successfully loaded " << states.size() << " states.";
  return states;
}

/// Write a vector of tracker states to a file.
auto writeTrackerStates(const std::vector<TrackerState>& states, const std::string& file_path) -> void {
  LOG(INFO) << "Writing tracking states (Format: t,x,y,theta,id) in file " << file_path << " ...";
  std::ofstream file(file_path);
  if (file.fail()) { LOG(ERROR) << "Error opening file. " << file_path; }
  file << std::fixed << std::showpoint;
  // Loop over lines parsing them as seeds.
  for (const auto& state : states) {
    file << state.t << "," << state.x << "," << state.y << "," << state.theta << "," << state.id << "\n";
  }
  file.close();
  LOG(INFO) << "Successfully written " << states.size() << " states.";
}

/// Initialize tracker centered given a full stream of events.
/// The event-stream will be forward and backward inspected to force an initialization as close as possible in time-space as the originating seed.
/// Initialization might fail if not enough events are found before and after the initial seed.
// TODO: Employ (templated) iterators instead of the vector per se.
auto initializeTrackerCentered(std::vector<Event>& events, Tracker& tracker) -> std::vector<Event>::iterator {

  CHECK_EQ(tracker.status(), haste::HypothesisPatchTracker::kUninitialized);
  CHECK_EQ(tracker.event_counter(), 0);// It is not half-initialized

  // Roll events
  auto it = std::lower_bound(events.begin(), events.end(), tracker.t(),
                             [](const Event& event, const Event::Time& time) { return event.t < time; });
  // 和下边的是不一样的，这里是++，下边是--
  // 这里是从0.6开始，一直到满了为止
  for (; it != events.end(); ++it) {
    const auto& event = *it;
    auto& [et, ex, ey, ep] = event;
    auto update_type = tracker.pushEvent(et, ex, ey);
    if (update_type == haste::HypothesisPatchTracker::kStateEvent) {
      break;// Populate tracker until it initializes
    }
  }

  if (it == events.end()) { LOG(ERROR) << "Event stream finished before feature is initialized."; }
  return it;
}

/// Initialize tracker non-centered given a full stream of events.
/// The event-stream will be processed forwards in time, initializing the tracker arbitrarily later than the original seed time.
/// Initialization might fail if not enough events are found after the initial seed.
// TODO: Employ (templated) iterators instead of the vector per se.
auto initializeTrackerRegular(std::vector<Event>& events, Tracker& tracker) -> std::vector<Event>::iterator {
  CHECK_EQ(tracker.status(), haste::HypothesisPatchTracker::kUninitialized);
  CHECK_EQ(tracker.event_counter(), 0);// It is not half-initialized

  // Gather half event-window from past events.
  // tracker.t is 0.6 also
  // std::cout<<tracker.t()<<std::endl;
  // lower_bound是查找啊，找处第一个大于0.6的时刻，到29636(shapes_translation)
  auto it_seed = std::lower_bound(events.begin(), events.end(), tracker.t(),
                                  [](const Event& event, const Event::Time& time) {return event.t < time; });
  // std::cout<<"loc: "<<it_seed-events.begin()<<std::endl;

  constexpr auto kEventWindowSizeHalf = Tracker::kEventWindowSize / 2;
  // 1+2*(0.2*31*31/2)=193
  // kEventWindowSizeHalf=193/2=96

  std::vector<Event> events_past_seed;// We could directly use iterators.
  for (auto it = it_seed; it != events.begin()
       && events_past_seed.size()< kEventWindowSizeHalf;// TODO: Wrong range (missing events.begin()). Use reverse iterator instead.
       --it) {                   // TODO: verify Range <= for window size
    const auto& event = *it;
    // 如果当前这个事件在追踪窗口内，那么才把他加入到events_past_seed
    if (tracker.isEventInRange(event.x, event.y)) { events_past_seed.push_back(event); }
  }

  if (events_past_seed.size() != kEventWindowSizeHalf) {
    LOG(ERROR) << "Not enough events before the provided seed to enable centered tracker initialization.";
    return events.end();
  }

  // Feed half of the event window.
  // 原本因为是it--，是按时间倒序，现在正过来
  std::reverse(events_past_seed.begin(), events_past_seed.end());
  // std::cout<<events_past_seed.size()<<std::endl;    //96

  // 理论上来说，events_past_seed有96个，而pushEvent内部的参数是193个，所以CHECK_EQ应该不会报错
  for (const auto& event_past_seed : events_past_seed) {
    auto& [et, ex, ey, ep] = event_past_seed;
    auto update_type = tracker.pushEvent(et, ex, ey);
    CHECK_EQ(update_type,
             haste::HypothesisPatchTracker::kInitializingEvent);// TODO: Improve naming instead of out of range
            // 似乎是验证失败,就终止运行,并写FATAL日志
  }

  // Proceed with the rest of the events.
  // 0.6s以后的
  // 这里很奇怪，为啥要这么做呢，初始化的数据，0.6秒前后各占一半
  // 这里的看上去就和另一个初始化函数差不多了
  auto it = it_seed;
  for (; it != events.end(); ++it) {
    auto& [et, ex, ey, ep] = *it;
    // 在这里边进行的初始化
    auto update_type = tracker.pushEvent(et, ex, ey);

    if (update_type == haste::HypothesisPatchTracker::kStateEvent) {
      break;// Out of image (with margin)
    }
  }
  // 终于把这里看完了,现在主要是把template_定下来了,并对当前的状态构造了11个假设

  if (it == events.end()) { LOG(ERROR) << "Event stream finished before tracker is initialized."; }
  return it;
}

}// namespace app
}// namespace haste