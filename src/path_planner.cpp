#include "path_planner.hpp"
#include "helpers.h"

#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

PathPlanner::PathPlanner(const points_2d_t & map_xy, const points_2d_t & map_dxdy, const points_1d_t & map_s, double delta_t) : map_xy_{map_xy}, map_dxdy_{map_dxdy}, map_s_{map_s}, delta_t_{delta_t} {}

PathPlanner::points_2d_t PathPlanner::getNextPath(const points_2d_t & previous_path, const point_t & path_end, const Pose & current_pose, sensor_fusion_t fusion_results) {
  if (previous_path.first.size() != previous_path.second.size()) {
    std::stringstream msg;
    msg << "Previous path vectors must be of equal length! (" << previous_path.first.size() <<
    " != " << previous_path.second.size() << ")";
    throw std::runtime_error(msg.str());
  }

  size_t prev_path_size = previous_path.first.size();
  if (prev_path_size > kNumPoints) {
    std::stringstream msg;
    msg << "Previous path has more points than expected! " << prev_path_size << " > " << kNumPoints;
    throw std::runtime_error(msg.str());
  }

  points_2d_t new_points;

  size_t num_new_points = kNumPoints - prev_path_size;
  point_t next_frenet{current_pose.car_s, current_pose.car_d};
  for (size_t i = 0; i < num_new_points; ++i) {
    current_acceleration_ = updateKinematics(current_acceleration_, target_acceleration_, kMaxJerk, delta_t_);
    current_velocity_ = updateKinematics(current_velocity_, target_velocity_, kMaxAcceleration, delta_t_);
    next_frenet = generatePointAheadFrenet(next_frenet, current_velocity_);
    auto xy = getXY(next_frenet.first, next_frenet.second, map_s_, map_xy_.first, map_xy_.second);
    new_points.first.push_back(xy[0]);
    new_points.second.push_back(xy[1]);
  }

  return new_points;
}

double PathPlanner::updateKinematics(double current, double target, double limit, double delta_t) {
  double increment = delta_t * limit;
  double error = target - current;
  double sign = (error > 0.0) ? 1 : -1;
  if (::fabs(error) < increment) {
    current = target;
  } else {
    current += increment * sign;
  }
  return current;
}

PathPlanner::point_t PathPlanner::generatePointAheadFrenet(point_t point, double velocity) {
  // Simply adjust s value by distance given by velocity and delta_t while keeping d constant
  return {point.first + velocity * delta_t_, point.second};
}
