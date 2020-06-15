#include "path_planner.hpp"
#include "helpers.h"

#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

PathPlanner::PathPlanner(const points_2d_t & map_xy, const points_2d_t & map_dxdy, const points_1d_t & map_s, double delta_t) : map_xy_{map_xy}, map_dxdy_{map_dxdy}, map_s_{map_s}, delta_t_{delta_t} {}

PathPlanner::points_2d_t PathPlanner::getNextPath(const points_2d_t & previous_path, const point_2d_t & path_end, const Pose & current_pose, sensor_fusion_t fusion_results) {
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

  auto spline = generateSpline(previous_path, current_pose);
  points_2d_t new_points = generateNextPathFromSpline(spline, previous_path, current_pose);

  // std::cout << "NEW POINTS" << std::endl;
  // for (size_t i = 0; i < new_points.first.size(); ++i) {
  //   std::cout << "x = " << new_points.first[i] << " y = " << new_points.second[i] << std::endl;
  // }

  /*
  size_t num_new_points = kNumPoints - prev_path_size;
  point_2d_t next_frenet{current_pose.car_s, current_pose.car_d};
  for (size_t i = 0; i < num_new_points; ++i) {
    current_acceleration_ = updateKinematics(current_acceleration_, target_acceleration_, kMaxJerk, delta_t_);
    current_velocity_ = updateKinematics(current_velocity_, target_velocity_, kMaxAcceleration, delta_t_);
    next_frenet = generatePointAheadFrenet(next_frenet, current_velocity_);
    auto point = getXYFromFrenet(next_frenet);
    new_points.first.push_back(point.first);
    new_points.second.push_back(point.second);
  }
  */

  return new_points;
}

tk::spline PathPlanner::generateSpline(const points_2d_t & previous_path, const Pose & current_pose) {
  // First let's gather some points for our spline. We'll use the previous two
  // points and then project a few more points out into the distance along
  // our current lane
  static constexpr size_t num_prev_points{2};
  points_2d_t input_points_xy;
  auto prev_path_size = previous_path.first.size();

  const auto & x = previous_path.first;
  const auto & y = previous_path.second;
  if (prev_path_size >= num_prev_points) {
    for (size_t i = num_prev_points; i > 0; --i) {
      input_points_xy.first.push_back(x[prev_path_size-i]);
      input_points_xy.second.push_back(y[prev_path_size-i]);
    }
  } else {
    // Use two points that make the path tangent to the car's current pose
    double prev_car_x = current_pose.car_x - ::cos(current_pose.car_yaw);
    double prev_car_y = current_pose.car_y - ::sin(current_pose.car_yaw);

    input_points_xy.first.push_back(prev_car_x);
    input_points_xy.first.push_back(current_pose.car_x);
    input_points_xy.second.push_back(prev_car_y);
    input_points_xy.second.push_back(current_pose.car_y);
  }

  // std::cout << "POSE X = " << current_pose.car_x << " Y = " << current_pose.car_y << " YAW = " << current_pose.car_yaw << std::endl;

  // At this point we have our first two points, let's generate a few more
  static constexpr double increment = 30.0;
  for (size_t i = 0; i < 3; ++i) {
    double s = current_pose.car_s + (i + 1) * increment;
    double d = getLaneCenterPositionOfCurrentLane(current_pose);
    // std::cout << "MAP s = " << s << " d = " << d << std::endl;
    point_2d_t point{s, d};
    auto waypoint = getXYFromFrenet(point);
    input_points_xy.first.push_back(waypoint.first);
    input_points_xy.second.push_back(waypoint.second);
  }

  // Now we have all of our points, let's transform to car coordinate frame
  // to make the spline generation much simpler...
  for (size_t i = 0; i < input_points_xy.first.size(); ++i) {

    auto x = input_points_xy.first[i];
    auto y = input_points_xy.second[i];
    // std::cout << "MAP x = " << x << " y = " << y << std::endl;
    auto xy_transform = mapFrameToCarFrame({x, y}, current_pose);
    input_points_xy.first[i] = xy_transform.first;
    input_points_xy.second[i] = xy_transform.second;
    // std::cout << "CAR x = " << xy_transform.first << " y = " << xy_transform.second << std::endl;
  }

  tk::spline s;
  s.set_points(input_points_xy.first, input_points_xy.second);
  return s;
}

PathPlanner::points_2d_t PathPlanner::generateNextPathFromSpline(const tk::spline & s, const points_2d_t & previous_path, const Pose & current_pose) {
  points_2d_t next_path;
  next_path.first.insert(next_path.first.begin(), previous_path.first.begin(), previous_path.first.end());
  next_path.second.insert(next_path.second.begin(), previous_path.second.begin(), previous_path.second.end());

  size_t prev_path_size = previous_path.first.size();
  size_t num_new_points = kNumPoints - prev_path_size;

  if (!num_new_points) {
    return next_path;
  }

  // We are in car reference frame, so our heading is along the x axis.
  // We want to choose an arbitrary distance horizon along
  // our X axis, which we then use to find a target point along the spline.

  // Now that we have our target point, we can figure out the length of our
  // discrete distance increments to reach the point at our desired velocity.
  double target_x = kSplineDistanceHorizon;
  double target_y = s(target_x);
  double distance = ::sqrt(target_x*target_x + target_y*target_y);

  // std::cout << "NUMBER OF NEW POINTS " << num_new_points << std::endl;

  double next_x = 0.0;
  if (prev_path_size > 0) {
    // let's start where we last left off
    point_2d_t last{previous_path.first[prev_path_size-1], previous_path.second[prev_path_size-1]};
    last = mapFrameToCarFrame(last, current_pose);
    next_x = last.first;
    // std::cout << "STARTING FROM X = " << next_x << std::endl;
  }
  for (size_t i = 0; i < num_new_points; ++i) {
    // Number of increments along the hypotnuse
    current_acceleration_ = updateKinematics(current_acceleration_, target_acceleration_, kMaxJerk, delta_t_);
    current_velocity_ = updateKinematics(current_velocity_, target_velocity_, kMaxAcceleration, delta_t_);
    double N = distance / (delta_t_ * current_velocity_);
    // Distance of each increment along the X axis
    double x_dist_increment = target_x / N;
    next_x += x_dist_increment;
    const double next_y = s(next_x);


    auto xy_transform = carFrameToMapFrame({next_x, next_y}, current_pose);

    // std::cout << "NEW x = " << xy_transform.first << " y = " << xy_transform.second << std::endl;

    next_path.first.push_back(xy_transform.first);
    next_path.second.push_back(xy_transform.second);
  }

  return next_path;
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

PathPlanner::point_2d_t PathPlanner::generatePointAheadFrenet(point_2d_t point, double velocity) {
  // Simply adjust s value by distance given by velocity and delta_t while keeping d constant
  return {point.first + velocity * delta_t_, point.second};
}

double PathPlanner::getLaneCenterPositionOfCurrentLane(const Pose & current_pose) {
  double d = current_pose.car_d;
  unsigned int lane_no = 0;
  while (d > kLaneWidth) {
    d -= kLaneWidth;
    lane_no++;
  }
  return (lane_no * kLaneWidth) + (kLaneWidth / 2.0);
}

PathPlanner::point_2d_t PathPlanner::getXYFromFrenet(point_2d_t frenet) {
  auto xy = getXY(frenet.first, frenet.second, map_s_, map_xy_.first, map_xy_.second);
  return {xy[0], xy[1]};
}

PathPlanner::point_2d_t PathPlanner::mapFrameToCarFrame(point_2d_t map_coord, const Pose & current_pose) {
  auto & x = map_coord.first;
  auto & y = map_coord.second;
  auto & origin_x = current_pose.car_x;
  auto & origin_y = current_pose.car_y;
  auto & origin_yaw = current_pose.car_yaw;
  double shift_x = x - origin_x;
  double shift_y = y - origin_y;

  x = (shift_x * ::cos(-origin_yaw) - shift_y * ::sin(-origin_yaw));
  y = (shift_x * ::sin(-origin_yaw) + shift_y * ::cos(-origin_yaw));
  return {x, y};
}

PathPlanner::point_2d_t PathPlanner::carFrameToMapFrame(point_2d_t car_coord, const Pose & current_pose) {
  double x_ref = car_coord.first;
  double y_ref = car_coord.second;
  double ref_yaw = current_pose.car_yaw;

  double x = x_ref * ::cos(ref_yaw) - y_ref * ::sin(ref_yaw);
  double y = x_ref * ::sin(ref_yaw) + y_ref * ::cos(ref_yaw);

  x += current_pose.car_x;
  y += current_pose.car_y;
  return {x, y};
}
