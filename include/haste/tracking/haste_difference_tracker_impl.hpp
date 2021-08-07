// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
namespace haste {

HasteDifferenceTracker::HasteDifferenceTracker(const Time &t, const Location &x, const Location &y,
                                                           const Orientation &theta)
    : HypothesisPatchTracker(t, x, y, theta) {}

auto HasteDifferenceTracker::trackerName() const -> std::string { return "HasteDifferenceTracker"; };

auto HasteDifferenceTracker::updateTemplate() -> void {
  updateTemplateWithMiddleEvent(kWeight_);
};

auto HasteDifferenceTracker::eventWindowToModel(const EventWindow &event_window,
                                                      const Hypothesis &hypothesis) const -> Patch {
  return eventWindowToModelUnitary(event_window, hypothesis, kWeight_);
}

auto HasteDifferenceTracker::initializeHypotheses() -> void {
  // 居然到了计算假设分数的地方了
  // kNumHypotheses is 11
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    hypotheses_score_[i] = -getDifferencePatch_(hypotheses_[i]).square().sum();
  }
}

auto HasteDifferenceTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  const auto &[t_old, x_old, y_old] = oldest_event;
  const auto &[t_new, x_new, y_new] = newest_event;

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    // 这个是公式7那部分的吧
    // 将矩阵上的每一位平方，在将每一位相加
    hypotheses_score_[i] = -getDifferencePatch_(hypotheses_[i]).square().sum();
    // 反正就是在这一步把假设的分数给计算出来了
  }
}

auto HasteDifferenceTracker::getDifferencePatch_(const Hypothesis &hypothesis) -> Patch {
  const auto hypothesis_model = eventWindowToModel(event_window_, hypothesis);
  // 为啥要kEventWindowSize*kWeight呢，这不就得1了吗
  // 减号前边 就是(7)归一化后的模板
  Patch difference_patch = template_ / template_.sum() - hypothesis_model / (kEventWindowSize * kWeight_);
  // 返回的是一个31*31的Eigen矩阵
  return difference_patch;
}

}// namespace haste