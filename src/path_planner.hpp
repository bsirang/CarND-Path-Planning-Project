#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <utility>
#include <vector>

#include "spline.h"

#include "behavior_planner.hpp"
#include "common_types.hpp"

class PathPlanner {
public:

  using Object = Common::Object;
  using Pose = Common::Pose;
  using lane_t = Common::lane_t;
  using points_2d_t = Common::points_2d_t;
  using points_1d_t = Common::points_1d_t;
  using point_2d_t = Common::point_2d_t;
  using sensor_fusion_t = Common::sensor_fusion_t;

  PathPlanner(const points_2d_t & map_xy, const points_2d_t & map_dxdy, const points_1d_t & map_s, double delta_t);

  points_2d_t getNextPath(const points_2d_t & previous_path, const point_2d_t & path_end, const Pose & current_pose, sensor_fusion_t fusion_results);

public: // static utility methods
  static void sanityCheckPreviousPath(const points_2d_t & previous_path);

private: // methods

  point_2d_t generatePointAheadFrenet(point_2d_t point, double velocity) const;
  point_2d_t getXYFromFrenet(point_2d_t frenet) const;

  tk::spline generateSpline(const points_2d_t & previous_path, const Pose & current_pose, lane_t target_lane);
  points_2d_t generateNextPathFromSpline(const tk::spline & s, const points_2d_t & previous_path, const Pose & current_pose);
  void proximityDetector(const std::vector<Object> & objects, const Pose & current_pose);
  double calculateBrakingDistance();

private: // members
  const points_2d_t map_xy_;
  const points_2d_t map_dxdy_;
  const points_1d_t map_s_;
  const double delta_t_;

  double target_velocity_{Common::kMaxVelocity};
  double current_velocity_{0.0};

  double target_acceleration_{Common::kMaxAcceleration};
  double current_acceleration_{0.0};

  BehaviorPlanner bp_{Common::kInitialLane};
  lane_t last_target_lane_{Common::kInitialLane};
};

#endif // PATH_PLANNER_HPP
