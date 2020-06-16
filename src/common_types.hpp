#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <cmath>
#include <utility>
#include <vector>

namespace Common {
struct Object {
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

struct Pose {
  Pose(double x, double y, double s, double d, double yaw, double speed) : x{x}, y{y}, s{s}, d{d}, yaw{yaw}, speed{speed} {}
  explicit Pose(const Object & o) : x{o.x}, y{o.y}, s{o.s}, d{o.d}, yaw{0.0}, speed{::sqrt(o.vx*o.vx+o.vy*o.vy)} {}
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

using lane_t = unsigned int;
using points_2d_t = std::pair<std::vector<double>, std::vector<double>>;
using points_1d_t = std::vector<double>;
using point_2d_t = std::pair<double, double>;
using sensor_fusion_t = std::vector<std::vector<double>>;


constexpr double kMphToMs = 2.23694;
constexpr unsigned kNumPoints = 50;
constexpr double kMaxVelocity = 49.5 / kMphToMs;
constexpr double kMaxAcceleration = 9.5; // m/s^2
constexpr double kMaxJerk = 5; // m/s^3
constexpr double kLaneWidth = 4; // m/s
constexpr double kSplineAnchorDistanceInterval = 60.0; // m
constexpr unsigned kSplineNumPoints = 3;
constexpr double kSplineDistanceHorizon = 30.0; // m
constexpr double kCarWidth = 3.0; // m
constexpr lane_t kInitialLane = 1; // from simulator
constexpr double kBrakingMargin = 10.0; // m

constexpr double kProjectAheadTime = 3.0; // sec
constexpr double kOccupancyWindow = 6.0; // m
constexpr double kObjectAheadThreshold = 30.0; // m
constexpr double kLaneChangeBaseCost = 0.10;
constexpr double kLaneOccupiedBaseCost = 0.75;
constexpr unsigned kPrepareChangeThreshold = 7;
}

#endif // COMMON_TYPES_HPP
