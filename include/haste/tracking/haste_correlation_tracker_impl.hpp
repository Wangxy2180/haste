// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {
HasteCorrelationTracker::HasteCorrelationTracker(const Time &t, const Location &x, const Location &y,
                                                 const Orientation &theta)
    : CorrelationTracker(t, x, y, theta) {}

auto HasteCorrelationTracker::trackerName() const -> std::string { return "HasteCorrelationTracker"; };

auto HasteCorrelationTracker::appendEventToWindow(const EventTuple &newest_event) -> EventTuple {
  const auto oldest_event = event_window_.appendEvent(newest_event);
  
  // 原始的只有上边那一句和最后一句，其他的都是特有的
  // TODO: subs with an actual rolling vector efficiency
  // 这个stack本质上也是一个array,11*193个大小
  // 本质上就是将旧事件的11个给去掉了
    // std::cout<<"app1"<<samples_stack_(1,1)<<std::endl;

  samples_stack_.leftCols<kEventWindowSize - 1>() = samples_stack_.rightCols<kEventWindowSize - 1>();

  const auto &[et, ex, ey] = newest_event;
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    const auto &[xp, yp] = patchLocation(ex, ey, hypothesis);
    // 4->1
    const auto &sampled_value = Interpolator::bilinearSample(template_, xp, yp);
    // 后续也没看到这个samples_stack有啥特殊的操作啊
    samples_stack_(i, kEventWindowSize - 1) = sampled_value;
  }
  // 这个samples_stack存放了所有的11个假设的193个坐标所对应的模板权重参数
    // std::cout<<"app2"<<samples_stack_(1,1)<<std::endl;
  event_counter_++;
  return oldest_event;
}

// init只在每次假设更新时会用到
// 应该是因为假设更新后，位置会发生变化，所以需要重新计算xp_vec
auto HasteCorrelationTracker::initializeHypotheses() -> void {
  // kNumHypotheses is 11
  // 我咋觉得和前边没啥不同呢,
  // 但是在correla中，因为本质上和updateScore没啥不同，所以用了几次
    // std::cout<<"-----ini1"<<samples_stack_(1,1)<<std::endl;

    // 这里确实会发生变化，但是为啥呢
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    // 那这一步完全是多余啊,因为samples_stack中不是已经存放了从template中取到的值吗
    const auto &hypothesis = hypotheses_[i];
    const auto &[xp_vec, yp_vec] = patchLocation(event_window_.ex_vec(), event_window_.ey_vec(), hypothesis);
    const auto &sampled_values = Interpolator ::bilinearSampleVector(template_, xp_vec, yp_vec);
    // 我咋觉得这个samples_stack是多次一举呢,明明这里也是直接替换了整个row啊
    // 可能就是算分好算?
    samples_stack_.row(i) = sampled_values;
  }
    // std::cout<<"-----ini2"<<samples_stack_(1,1)<<std::endl;
  
  hypotheses_score_ = (samples_stack_.matrix() * weights_.matrix()).array();
};

auto HasteCorrelationTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  // Ignore newest and oldest event and proceed from scratch
    // std::cout<<"upd"<<samples_stack_(1,1)<<std::endl;
  hypotheses_score_ = (samples_stack_.matrix() * weights_.matrix()).array();
};

}// namespace haste