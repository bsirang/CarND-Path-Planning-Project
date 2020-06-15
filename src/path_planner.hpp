#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <utility>
#include <vector>
#include <cmath>

#include "spline.h"

class PathPlanner {
public:
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

  using points_2d_t = std::pair<std::vector<double>, std::vector<double>>;
  using points_1d_t = std::vector<double>;
  using point_2d_t = std::pair<double, double>;
  using sensor_fusion_t = std::vector<std::vector<double>>;

  PathPlanner(const points_2d_t & map_xy, const points_2d_t & map_dxdy, const points_1d_t & map_s, double delta_t);

  points_2d_t getNextPath(const points_2d_t & previous_path, const point_2d_t & path_end, const Pose & current_pose, sensor_fusion_t fusion_results);

private: // constants
  static constexpr double kMphToMs = 2.23694;
  static constexpr unsigned kNumPoints = 50;
  static constexpr double kMaxVelocity = 49.5 / kMphToMs;
  static constexpr double kMaxAcceleration = 5; // m/s^2
  static constexpr double kMaxJerk = 5; // m/s^3
  static constexpr double kLaneWidth = 4; // m/s
  static constexpr double kSplineAnchorDistanceInterval = 30.0; // m
  static constexpr unsigned kSplineNumPoints = 3;
  static constexpr double kSplineDistanceHorizon = kSplineNumPoints * kSplineAnchorDistanceInterval; // m
  static constexpr double kBrakingMargin = 0.0;

private: // methods
  static double updateKinematics(double current, double target, double limit, double delta_t);
  static unsigned getCurrentLaneNo(const Pose & current_pose);
  static double getLaneCenterPositionOfCurrentLane(const Pose & current_pose);
  static double getCenterPositionOfLane(unsigned int lane_no);
  static point_2d_t mapFrameToCarFrame(point_2d_t map_coord, const Pose & current_pose);
  static point_2d_t carFrameToMapFrame(point_2d_t car_coord, const Pose & current_pose);

  point_2d_t generatePointAheadFrenet(point_2d_t point, double velocity) const;
  point_2d_t getXYFromFrenet(point_2d_t frenet) const;

  tk::spline generateSpline(const points_2d_t & previous_path, const Pose & current_pose);
  points_2d_t generateNextPathFromSpline(const tk::spline & s, const points_2d_t & previous_path, const Pose & current_pose);
  void proximityDetector(const std::vector<Object> & objects, const Pose & current_pose);
  double calculateBrakingDistance();

private: // members
  const points_2d_t map_xy_;
  const points_2d_t map_dxdy_;
  const points_1d_t map_s_;
  const double delta_t_;

  double target_velocity_{kMaxVelocity};
  double current_velocity_{0.0};

  double target_acceleration_{kMaxAcceleration};
  double current_acceleration_{0.0};

  unsigned int desired_lane_{1};
};

#endif // PATH_PLANNER_HPP
