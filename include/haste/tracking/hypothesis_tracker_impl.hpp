// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
// static bool flag_stop = true;

namespace haste {

HypothesisPatchTracker::HypothesisPatchTracker(const Time &t, const Location &x, const Location &y,
                                               const Orientation &theta) {
  hypotheses_[kNullHypothesisIdx] = Hypothesis{t, x, y, theta};
}

auto HypothesisPatchTracker::isEventInRange(const Location &ex, const Location &ey) const -> bool {
  //      auto dx = ex - x_;
  //      auto dy = ey - y_;
  //      auto xp = +dx * ctheta_ + dy * stheta_ + kPatchSizeHalf;
  //      auto yp = -dx * stheta_ + dy * ctheta_ + kPatchSizeHalf;
  // d2 rule
  // x() y()是state的x(),y(),也就是空假设的
  Location dx = ex - x();
  Location dy = ey - y();
  // kPatchSizeHalf 15
  constexpr auto d2_thresh = kPatchSizeHalf * kPatchSizeHalf;
  // 勾股
  // 也就是说他是判断是不是在这个半径15的圆的范围内
  return (dx * dx + dy * dy) < d2_thresh;
}

auto HypothesisPatchTracker::patchLocation(const Location &ex, const Location &ey, const Hypothesis &state) const
    -> std::pair<Location, Location> {
  const auto dx = ex - state.x();
  const auto dy = ey - state.y();
  const auto ctheta = state.ctheta();
  const auto stheta = state.stheta();

  auto xp = +dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
  auto yp = -dx * stheta + dy * ctheta + kPatchSizeHalf;
  return {xp, yp};
}

// 这个函数的最后一个输入，就是一个假设，也就是说state就是当前的null假设
template<int N>
auto HypothesisPatchTracker::patchLocation(const LocationVector<N> &ex_vec, const LocationVector<N> &ey_vec,
                                           const Hypothesis &state) const
    -> std::pair<LocationVector<N>, LocationVector<N>> {
      // 这里应该是对应HASTE3.1部分第二段倒数第二行那个公式
      // 或者是公式3？没有加weight的部分？
  // dx,dy是以patch中心为原点的坐标值
  const LocationVector<N> dx = ex_vec - state.x();
  const LocationVector<N> dy = ey_vec - state.y();
  // cos(theta)和sin(theta)
  const auto ctheta = state.ctheta();
  const auto stheta = state.stheta();

  // 这里是转置过后的旋转矩阵
  // 所以计算过后，变成了2*1的矩阵，其中每一横行还是193*1
  // kPatchSizeHalf=15
  // 这里为啥给他相加呢？转置之后就是在patch中，以中心为原点，右、下为正半轴的坐标
  // 这里可以见附件excel，它变成了以patch左上为原点的坐标
  LocationVector<N> xp_vec = dx * ctheta + dy * stheta + kPatchSizeHalf;// Patch coordinates
  LocationVector<N> yp_vec = -dx * stheta + dy * ctheta + kPatchSizeHalf;

  // 以中心为原点，右、下为正半轴的坐标
  // TODO: verify the return type does not harm Eigen alignment
  return {xp_vec, yp_vec};
}

auto HypothesisPatchTracker::updateTemplateWithMiddleEvent(const Weight &weight) -> void {
  const auto &[et, ex, ey] = event_window_.middleEvent();
  const auto [xp, yp] = patchLocation(ex, ey, state());
  // MH第五页左上角
  // 这里就是重新计算中心点所占的权重，这个意思是不是说，后续用作模板的元素，他的权重会变低 1->4
  // 其实这里也好理解，就是一个模板权重增量
  Interpolator::bilinearIncrementVector(template_, xp, yp, weight * kTemplateUpdateFactor);
}

// 和下边vector的不同之处仅在于weight的不同
// 这个unitary和vector应该就是指权重的形式
auto HypothesisPatchTracker::eventWindowToModelUnitary(const EventWindow &event_window, const Hypothesis &hypothesis,
                                                       const Weight &weight) const -> Patch {
  // Patch 31*31
  Patch model = Patch::Zero();
  // 分别获取所有的x和y值，也就是将其放在两个向量中
  EventWindowLocationVector ex_vec = event_window.ex_vec();
  EventWindowLocationVector ey_vec = event_window.ey_vec();
  // 这里得到的是经过旋转矩阵的193*2
  // 是在patch中以左上角为原点的坐标值,从0开始
  const auto [xp_vec, yp_vec] = patchLocation(ex_vec, ey_vec, hypothesis);
  // kEventWindowSize is 193
  for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
    const Location &xp = xp_vec[i];
    const Location &yp = yp_vec[i];
    // 权重1/193 1->4
    Interpolator::bilinearIncrementVector(model, xp, yp, weight);
  }
  // 根据每个被激发的事件点，在其附近4个点上插值
  return model;
}

auto HypothesisPatchTracker::eventWindowToModelVector(const EventWindow &event_window, const Hypothesis &hypothesis,
                                                      const EventWindowVector<Weight> &weights) const -> Patch {
  Patch model = Patch::Zero();
  EventWindowLocationVector ex_vec = event_window.ex_vec();
  EventWindowLocationVector ey_vec = event_window.ey_vec();
  
  // 对当前状态进行旋转后的坐标，float
  auto [xp_vec, yp_vec] = patchLocation(ex_vec, ey_vec, hypothesis);

  for (size_t i = 0; i < kEventWindowSize; ++i) {// TODO(ialzugaray): "vectorizable" loop with Eigen binaryExpr
    const Location &xp = xp_vec[i];
    const Location &yp = yp_vec[i];
    const Weight &weight = weights[i];
    // 他和其他的插值方法，只有这个权重的设置不同，其他的用的都是固定的1/193，只有这个是高斯权重 1->4
    Interpolator::bilinearIncrementVector(model, xp, yp, weight);
  }
  // 初始化到这里，是template采集结束
  // 虽然他叫model,但似乎除了可视化部分,他只用作template初始化了
  return model;
}

auto HypothesisPatchTracker::initializeTracker() -> void {
  status_ = TrackerStatus::kRunning;
  // middle is No.96
  const auto &[et, ex, ey] = event_window_.middleEvent();
  // std::cout<<event_window_.ey_vec()<<std::endl;
  // std::cout<<event_window_.ex_vec()<<std::endl;
  // x,y,theta就是在运行时输入的初始值
  // 这个假设，应该和状态state是一个意思，都是用一个xyt theta定义的点
  // 这个假设，就是一个存储xytr的类
  Hypothesis initial_hypothesis{et, x(), y(), theta()};
  // 下边返回的是model，是根据所有被激发的事件，通过插值创造的模板
  // 而且这个函数只有在可视化和创造模板时用到
  // template_ 31*31，用193个坐标和对应的权重1->4插值得到
  template_ = eventWindowToModel(event_window_, initial_hypothesis);
  // 将当前的状态转换为11个扰动状态，并进行评分，存到hypotheses_score_中
  transitionToHypothesis(initial_hypothesis);
  // 这里和追踪的步骤很像，追踪也是先获取中间的时间，然后更新假设并更新分数
}

auto HypothesisPatchTracker::appendEventToWindow(const EventTuple &newest_event) -> EventTuple {
  const auto oldest_event = event_window_.appendEvent(newest_event);
  event_counter_++;
  return oldest_event;
}

auto HypothesisPatchTracker::updateHypothesesTimeFromMiddleEvent() {
  // middleEvent is No.96
  auto [et_mid, ex_mid, ey_mid] = event_window_.middleEvent();
  // 对于所有的11个假设，用事件窗口中心的时间更新假设的时间
  for (auto &hypothesis : hypotheses_) { hypothesis.t() = et_mid; }
}
auto HypothesisPatchTracker::pushEvent(const Time &et, const Location &ex, const Location &ey) -> EventUpdate {
  if (//(et <= t()) ||
  // 这里还要再检查一遍，初始化的时候检查过一次，这里应该是给后边留的
  // 检查是不是在state的template范围内
      (!isEventInRange(ex, ey))) {
    return kOutOfRange;
  }

  EventTuple newest_event{et, ex, ey};
  // 这里就是文中说的，用新事件代替旧事件
  // 不同tracker实现有不同
  const auto oldest_event = appendEventToWindow(newest_event);
  
  // 第一次初始化运行时，这里是true，初始化过后，就再也不会走这里了
  if (status_ == kUninitialized) {
    if (event_counter_ >= kEventWindowSize) {// TODO: verify >= range
    // 直到event_window里满足了条件，才会走这条路
    // 其实这里是初始化模板和假设
      initializeTracker();
      return kStateEvent;
    } else {
      // 正常情况下,初始化模板时应该走这里
      return kInitializingEvent;
    }
  }
  
  // 使用新的事件窗口中心的时间去更新每个假设的时间
  // 似乎只有在记录的时候才会用到更新的时间
  updateHypothesesTimeFromMiddleEvent();// TODO: Not relevant until change of state;
  // 因为event_window产生了变化，所以需要
  // 更新每个假设的分数
  updateHypothesesScore(oldest_event, newest_event);

  // 寻找分数最高的
  auto best_hypothesis_idx = getBestHypothesisIdx();
  EventUpdate ret;
  // 如果假设没变。那就返回常规事件，如果变了，那就返回状态事件，
  // 也就是说这个事件，到底会不会导致他的状态发生改变
  if (best_hypothesis_idx == kNullHypothesisIdx) {
    ret = EventUpdate::kRegularEvent;
  } else {
    ret = EventUpdate::kStateEvent;
    // 根据更新的状态，产生11个假设
    transitionToHypothesis(hypotheses_[best_hypothesis_idx]);
  }

  // template的update 就是计算权重增量
  // 这个函数所有tracker都是一样的
  updateTemplate();
  return ret;
}

auto HypothesisPatchTracker::getBestHypothesisIdx() const -> size_t {
  const auto &null_hypothesis_score = hypotheses_score_[kNullHypothesisIdx];
  size_t best_hypothesis_idx;
  // 分别是寻找矩阵中的最大值和最小值
  auto best_hypothesis_score = hypotheses_score_.maxCoeff(&best_hypothesis_idx);
  // TODO: Compute normalized versions only if null < best
  auto worst_hypothesis_score = hypotheses_score_.minCoeff();
  // 最大假设分数归一化，不就是1吗
  const auto best_hypothesis_score_normalized =
      (best_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);
  const auto null_hypothesis_score_normalized =
      (null_hypothesis_score - worst_hypothesis_score) / (best_hypothesis_score - worst_hypothesis_score);
      
  // 将假设的最大分数和空假设的最大分数进行比对，
  // 如果空假设的分数大于等于假设的分数，那么假设状态就不变，最新的分数就是未经假设的那个
  // 否则，如果有假设大于他了，，并且正则化后大于幅度超过0.05(HASTE 3.1最后一段 5%)，那么就要更新这个假设了
  if ((null_hypothesis_score < best_hypothesis_score)
      && ((best_hypothesis_score_normalized - null_hypothesis_score_normalized) > kHysteresisFactor)) {
    return best_hypothesis_idx;
  } else {
    return kNullHypothesisIdx;
  }
}

auto HypothesisPatchTracker::transitionToHypothesis(const Hypothesis &hypothesis) -> void {
  // 下边这个是11邻域的那个
  hypotheses_ = HypothesesGenerator::GenerateCenteredHypotheses(hypothesis);// Renew hypotheses
  // 这里边会获取所有的假设的分数,存在hypotheses_score_中
  // 因为涉及计算score，所以每个tracker计算方式不同
  initializeHypotheses();
}

}// namespace haste
