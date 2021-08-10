// Copyright (c) 2020 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <cmath>

// 单纯的定义了一个假设类，没有啥特殊的功能，主要就是存储参数
namespace haste {
template <typename S, typename T = double> struct HypothesisTXYR {
  // s is float
  using Time = T;
  using Location = S;
  using Orientation = S;

  constexpr HypothesisTXYR(){};
  constexpr HypothesisTXYR(const Time &t, const Location &x, const Location &y,
                           const Orientation &theta)
      : t_{t}, x_{x}, y_{y}, theta_{theta}, ctheta_{std::cos(theta)},
        stheta_{std::sin(theta)} {};

  // 目前来看，这个增量就是为了创造其余的10个假设准备的
  struct Incremental {
    //    constexpr Incremental(){};
    constexpr Incremental(const Location &dx, const Location &dy,
                          const Orientation &dtheta)
        : dx_{dx}, dy_{dy}, dtheta_{dtheta} {};

    //    const Location &dx() { return dx_; };
    //    const Location &dy() { return dy_; };
    //    const Orientation &dtheta() { return dtheta_; };
    Location dx() const { return dx_; };
    Location dy() const { return dy_; };
    Orientation dtheta() const { return dtheta_; };

    constexpr Incremental operator+(const Incremental &increment) const {
      return {dx() + increment.dx(), dy() + increment.dy(),
              dtheta() + increment.dtheta_};
    }

  protected:
    Location dx_, dy_;
    Orientation dtheta_;
  };//end Incremental

  const Time &t() const { return t_; };
  const Location &x() const { return x_; };
  const Location &y() const { return y_; };
  const Orientation &theta() const { return theta_; };
  const Orientation &ctheta() const { return ctheta_; };
  const Orientation &stheta() const { return stheta_; };

  Time &t() { return t_; };

  //  Time t() const { return t_; };
  //  Location x() const { return x_; };
  //  Location y() const { return y_; };
  //  Orientation theta() const { return theta_; };
  //  Orientation ctheta() const { return ctheta_; };
  //  Orientation stheta() const { return stheta_; };
  // 这里重载的+increment，时间不变，增量改变
  // 假设的加法确实就应该只加这三个
  constexpr HypothesisTXYR<S, T> operator+(const Incremental &increment) const {
    return {t(), x() + increment.dx(), y() + increment.dy(),
            theta() + increment.dtheta()};
  }

protected:
  Time t_;
  Location x_, y_;
  Orientation theta_, ctheta_, stheta_;
};
} // namespace haste
