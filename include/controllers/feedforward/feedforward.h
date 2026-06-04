#pragma once

#include <utility>

#include "Eigen/Dense"

struct FeedforwardOutput {
  double uLinear = 0.0;
  double uAngular = 0.0;
};

class Feedforward {
 private:
  // [kV, kA] gain vectors
  Eigen::RowVector2d linearGains;
  Eigen::RowVector2d angularGains;

 public:
  Feedforward()
      : linearGains(Eigen::RowVector2d::Zero()),
        angularGains(Eigen::RowVector2d::Zero()) {}

  Feedforward(const Eigen::RowVector2d& linearGains,
              const Eigen::RowVector2d& angularGains)
      : linearGains(linearGains), angularGains(angularGains) {}

  [[nodiscard]] FeedforwardOutput calculate(
      const double velocity, const double acceleration,
      const double angularVelocity, const double angularAcceleration) const {
    const double uLinear =
        linearGains * Eigen::Vector2d(velocity, acceleration);
    const double uAngular =
        angularGains * Eigen::Vector2d(angularVelocity, angularAcceleration);

    return {uLinear, uAngular};
  }

  [[nodiscard]] Eigen::RowVector2d getLinearGains() const {
    return linearGains;
  }

  void setLinearGains(const Eigen::RowVector2d& gains) {
    this->linearGains = gains;
  }

  [[nodiscard]] Eigen::RowVector2d getAngularGains() const {
    return angularGains;
  }

  void setAngularGains(const Eigen::RowVector2d& gains) {
    this->angularGains = gains;
  }
};

namespace FeedforwardConstants {
inline Eigen::RowVector2d DEFAULT_LINEAR{0.550, 0.00275};
inline Eigen::RowVector2d DEFAULT_ANGULAR{0.0895, 0.0170};

inline Feedforward defaultFeedforward =
    Feedforward(DEFAULT_LINEAR, DEFAULT_ANGULAR);
};  // namespace FeedforwardConstants