// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

HasteDifferenceStarTracker::HasteDifferenceStarTracker(const Time &t, const Location &x, const Location &y,
                                                       const Orientation &theta)
    : HasteDifferenceTracker(t, x, y, theta) {}

auto HasteDifferenceStarTracker::trackerName() const -> std::string { return "HasteDifferenceStarTracker"; };

auto HasteDifferenceStarTracker::initializeHypotheses() -> void {
  for (size_t i = 0; i < kNumHypotheses; ++i) {
    difference_patches_[i] = getDifferencePatch_(hypotheses_[i]);
    hypotheses_score_[i] = -difference_patches_[i].square().sum();
  }
};

// 这里可能就是公式9 10了吧
auto HasteDifferenceStarTracker::updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event)
    -> void {
  const auto &[t_old, x_old, y_old] = oldest_event;
  const auto &[t_new, x_new, y_new] = newest_event;

  for (size_t i = 0; i < kNumHypotheses; ++i) {
    const auto &hypothesis = hypotheses_[i];
    auto &hypothesis_score = hypotheses_score_[i];
    // 这个patch是公式7中的那个差，也就是template和model的差值
    auto &difference_patch = difference_patches_[i];
    // 这个difference_patch 还真变了
    // 对应k+1那两项
    updateDifferences_(difference_patch, newest_event, hypothesis, hypothesis_score, -1.0);
    // 对应k-m+1那两项
    updateDifferences_(difference_patch, oldest_event, hypothesis, hypothesis_score, +1.0);
  }
}

auto HasteDifferenceStarTracker::updateDifferences_(Eigen::Ref<Patch> difference_patch, const EventTuple &event,
                                                    const Hypothesis &hypothesis, Scalar &score,
                                                    const Scalar &increment) -> void {
  const auto &[et, ex, ey] = event;
  const auto [xp, yp] = patchLocation(ex, ey, hypothesis);

    // std::cout<<"1:"<<difference_patch((int)xp,(int)yp)<<std::endl;


    // 如果是旧事件，这里就是k-m+1
    // 如果是新事件，这里就是k+1

  if (xp >= 0 && yp >= 0 && xp < (kPatchSize - 1) && yp < (kPatchSize - 1)) {
    // 用这个事件坐标，通过双线性插值，得到他周边2*2的kernel
    auto interp_kernel = Interpolator::bilinearKernel(xp, yp);
    
    // 尺寸是2*2，左上角点是该事件变换后的位置
    // 下边的开始对应公式10了 
    // 和上边的interp_kernel相辅相成，这个是从diff_patch中取出的，而上边那个只是每个点所占的权重
    Eigen::Ref<Eigen::Array<Scalar, 2, 2>> difference_patch_block = difference_patch.block<2, 2>((int) xp, (int) yp);

    // 简单的说，减去旧的影响，加上新的影响
    // 下边这两个，对应公式10中的12项，也就是加上k时刻新旧事件的影响
    // Remove the contribution of this interpolated locations
    score += difference_patch_block.square().sum();
    // Update the values of the difference patch in the interpolated locations
    // 下边的第一项乘第三项就是正常的计算模板时候的权重啊，第二项应该作为符号位放在前边

    // 就是给这patch更新了一下权重，如果是新事件，那无疑是要加上新事件的权重的，所以这里increment是-1
    // 这里是更新different_patch_的地方，更新完之后就是在这之前是D(k)，在这之后是D(k+1)
    // 其实就是按照公式9算的，只不过代码中是分两步算的
    difference_patch_block += interp_kernel * increment * kWeight_;

    // 下边对应公式10的34项
    // Re-apply the contribution of this interpolated locations
    score -= difference_patch_block.square().sum();
  }
}

}// namespace haste
