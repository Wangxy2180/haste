// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <Eigen/Core>
#include <array>
#include <iterator>

#include <haste/core/hypothesis.hpp>

namespace haste {

template<typename T>
struct IncrementalHypothesesGenerator_TXYR_4neigh_2rot {
  using Scalar = T;
  using Hypothesis = HypothesisTXYR<Scalar>;
  using IncrementalHypothesis = typename Hypothesis::Incremental;

  // 这里对应的应该是公式2下边那片,代码中只有7个,论文中是11个,
  static constexpr Scalar deltaX = 1.0f;
  static constexpr Scalar deltaY = 1.0f;
  static constexpr Scalar deltaTheta = 4.0 * M_PI / 180.0;

  static constexpr std::array<IncrementalHypothesis,7>  kIncrementalHypotheses{
      IncrementalHypothesis{+0.0, +0.0, +0.0},       IncrementalHypothesis{+deltaX, +0.0, +0.0},
      IncrementalHypothesis{-deltaX, +0.0, +0.0},    IncrementalHypothesis{+0.0, +deltaY, +0.0},
      IncrementalHypothesis{+0.0, -deltaY, +0.0},    IncrementalHypothesis{+0.0, +0.0, +deltaTheta},
      IncrementalHypothesis{+0.0, +0.0, -deltaTheta}};
  static constexpr size_t kNullHypothesisIdx = 0;
};

template<typename T>
struct IncrementalHypothesesGenerator_TXYR_8neigh_2rot {
  using Scalar = T;
  using Hypothesis = HypothesisTXYR<Scalar>;
  using IncrementalHypothesis = typename Hypothesis::Incremental;

  static constexpr Scalar deltaX = 1.0f;
  static constexpr Scalar deltaY = 1.0f;
  static constexpr Scalar deltaTheta = 4.0 * M_PI / 180.0;

  // 这个才是八邻域
  static constexpr std::array<IncrementalHypothesis,11> kIncrementalHypotheses{
      IncrementalHypothesis{+0.0, +0.0, +0.0},       IncrementalHypothesis{+deltaX, +0.0, +0.0},
      IncrementalHypothesis{-deltaX, +0.0, +0.0},    IncrementalHypothesis{+0.0, +deltaY, +0.0},
      IncrementalHypothesis{+0.0, -deltaY, +0.0},    IncrementalHypothesis{+deltaX, +deltaY, +0.0},
      IncrementalHypothesis{-deltaX, +deltaY, +0.0}, IncrementalHypothesis{-deltaX, -deltaY, +0.0},
      IncrementalHypothesis{+deltaX, -deltaY, +0.0}, IncrementalHypothesis{+0.0, +0.0, +deltaTheta},
      IncrementalHypothesis{+0.0, +0.0, -deltaTheta}};
  static constexpr size_t kNullHypothesisIdx = 0;
};

// 这里的type就是上边八邻域那个
template<typename IncrementalHypothesesGeneratorType>
struct CenteredHypothesesGenerator {
  using IncrementalHypothesesGenerator = IncrementalHypothesesGeneratorType;
  using Scalar = typename IncrementalHypothesesGenerator::Scalar;
  using Hypothesis = typename IncrementalHypothesesGenerator::Hypothesis;
  static constexpr auto kNullHypothesisIdx = IncrementalHypothesesGenerator::kNullHypothesisIdx;
  static constexpr auto kIncrementalHypotheses = IncrementalHypothesesGenerator::kIncrementalHypotheses;

  static constexpr auto kNumHypotheses = kIncrementalHypotheses.size();
  using Hypotheses = std::array<Hypothesis, kNumHypotheses>;

  static constexpr Hypotheses GenerateCenteredHypotheses(const Hypothesis &null_hypothesis) {
    Hypotheses hypotheses;
    // 看上去是讲一个假设变成了11个假设
    for (size_t i = 0; i < kNumHypotheses; ++i) {// TODO null hypothesis could be avoided
    // 这个+被重载了，这一层套一层的，真费劲
      hypotheses[i] = null_hypothesis + kIncrementalHypotheses[i];
    }
    return hypotheses;
  }
};

}// namespace haste
