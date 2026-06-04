#pragma once

#include <cmath>

#include "subsystems/drivetrain.h"

struct MotionProfileConstraints {
  double maxVelocity = 0.0;
  double maxAcceleration = 0.0;
};

class MotionProfile1D {
 private:
  MotionProfileConstraints constraints;

  double distance;
  double startVelocity;
  double endVelocity;
  double maxVelocity;

  double acceleration;
  double deceleration;

  double accelEndTime;
  double stableEndTime;
  double totalTime;

 public:
  MotionProfile1D(double distance, double startVelocity, double endVelocity,
                  const MotionProfileConstraints& constraints)
      : constraints(constraints),
        distance(distance),
        startVelocity(startVelocity),
        endVelocity(endVelocity) {};

  double getTotalTime() const { return totalTime; }

  double getDistanceAtTime(double time) const {
    if (time < accelEndTime) {
      return startVelocity * time + 0.5 * acceleration * time * time;
    } else if (time < stableEndTime) {
      const double accelDistance =
          startVelocity * accelEndTime +
          0.5 * acceleration * accelEndTime * accelEndTime;
      const double stableTime = time - accelEndTime;
      return accelDistance + maxVelocity * stableTime;
    } else if (time <= totalTime) {
      const double accelDistance =
          startVelocity * accelEndTime +
          0.5 * acceleration * accelEndTime * accelEndTime;
      const double stableDistance =
          maxVelocity * (stableEndTime - accelEndTime);
      const double decelTime = time - stableEndTime;
      return accelDistance + stableDistance + maxVelocity * decelTime -
             0.5 * deceleration * decelTime * decelTime;
    } else {
      return distance;
    }
  }
  double getVelocityAtTime(double time) const {
    if (time < accelEndTime) {
      return startVelocity + acceleration * time;
    } else if (time < stableEndTime) {
      return maxVelocity;
    } else if (time <= totalTime) {
      const double decelTime = time - stableEndTime;
      return maxVelocity - deceleration * decelTime;
    } else {
      return endVelocity;
    }
  }
  double getAccelerationAtTime(double time) const {
    if (time < accelEndTime) {
      return acceleration;
    } else if (time < stableEndTime) {
      return 0.0;
    } else if (time <= totalTime) {
      return -deceleration;
    } else {
      return 0.0;
    }
  }

  void generateProfile() {
    const double dir = (distance >= 0.0) ? 1.0 : -1.0;
    const double D = std::fabs(distance);
    const double a = constraints.maxAcceleration;
    const double vLim = constraints.maxVelocity;

    // Speed magnitudes along the travel direction.
    const double u0 = startVelocity * dir;
    const double uf = endVelocity * dir;

    if (a <= 0.0 || vLim <= 0.0) {
      // Degenerate constraints: no motion possible.
      maxVelocity = startVelocity;
      acceleration = 0.0;
      deceleration = 0.0;
      accelEndTime = 0.0;
      stableEndTime = 0.0;
      totalTime = 0.0;
      return;
    }

    double vpSquared = (2.0 * a * D + u0 * u0 + uf * uf) / 2.0;
    if (vpSquared < 0.0) vpSquared = 0.0;
    double cruiseMag = std::sqrt(vpSquared);

    cruiseMag = std::fmin(cruiseMag, vLim);
    cruiseMag = std::fmax(cruiseMag, std::fmax(u0, uf));

    const double accelDist = (cruiseMag * cruiseMag - u0 * u0) / (2.0 * a);
    const double decelDist = (cruiseMag * cruiseMag - uf * uf) / (2.0 * a);
    double cruiseDist = D - accelDist - decelDist;
    if (cruiseDist < 0.0) cruiseDist = 0.0;

    const double accelTime = std::fabs(cruiseMag - u0) / a;
    const double decelTime = std::fabs(cruiseMag - uf) / a;
    const double cruiseTime = (cruiseMag > 0.0) ? cruiseDist / cruiseMag : 0.0;

    maxVelocity = dir * cruiseMag;
    acceleration = dir * std::copysign(a, cruiseMag - u0);
    deceleration = dir * std::copysign(a, cruiseMag - uf);

    accelEndTime = accelTime;
    stableEndTime = accelEndTime + cruiseTime;
    totalTime = stableEndTime + decelTime;
  };

  const double getStartVelocity() const { return startVelocity; }
  void setStartVelocity(const double startVelocity) {
    this->startVelocity = startVelocity;
  }
  const double getEndVelocity() const { return endVelocity; }
  void setEndVelocity(const double endVelocity) {
    this->endVelocity = endVelocity;
  }
  const double getDistance() const { return distance; }
  void setDistance(const double distance) { this->distance = distance; }
  const MotionProfileConstraints getProfileConstraints() const {
    return constraints;
  }
  void setProfileConstraints(const MotionProfileConstraints constraints) {
    this->constraints = constraints;
  }
};