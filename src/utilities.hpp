#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "common_types.hpp"

namespace Common {
namespace Utilities {
namespace {
double updateKinematics(double current, double target, double limit, double delta_t) {
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

lane_t getCurrentLaneNo(const Pose & current_pose) {
  double d = current_pose.d;
  unsigned int lane_no = 0;
  while (d > kLaneWidth) {
    d -= kLaneWidth;
    lane_no++;
  }
  return lane_no;
}

double getCenterPositionOfLane(lane_t lane_no) {
  return (lane_no * kLaneWidth) + (kLaneWidth / 2.0);
}

double getLaneCenterPositionOfCurrentLane(const Pose & current_pose) {
  return getCenterPositionOfLane(getCurrentLaneNo(current_pose));
}

point_2d_t mapFrameToCarFrame(point_2d_t map_coord, const Pose & current_pose) {
  auto & x = map_coord.first;
  auto & y = map_coord.second;
  auto & origin_x = current_pose.x;
  auto & origin_y = current_pose.y;
  auto & origin_yaw = current_pose.yaw;
  double shift_x = x - origin_x;
  double shift_y = y - origin_y;

  x = (shift_x * ::cos(-origin_yaw) - shift_y * ::sin(-origin_yaw));
  y = (shift_x * ::sin(-origin_yaw) + shift_y * ::cos(-origin_yaw));
  return {x, y};
}

point_2d_t carFrameToMapFrame(point_2d_t car_coord, const Pose & current_pose) {
  double x_ref = car_coord.first;
  double y_ref = car_coord.second;
  double ref_yaw = current_pose.yaw;

  double x = x_ref * ::cos(ref_yaw) - y_ref * ::sin(ref_yaw);
  double y = x_ref * ::sin(ref_yaw) + y_ref * ::cos(ref_yaw);

  x += current_pose.x;
  y += current_pose.y;
  return {x, y};
}

std::vector<Object> getObjectVectorFromSensorFusion(const sensor_fusion_t & fusion_results) {
  std::vector<Object> objects;
  for (const auto & result : fusion_results) {
    Object o;
    o.id = result[0];
    o.x = result[1];
    o.y = result[2];
    o.vx = result[3];
    o.vy = result[4];
    o.s = result[5];
    o.d = result[6];
    objects.push_back(o);
    // std::cout << "id = " << o.id << " d = " << o.d << " speed = " << ::sqrt(o.vx*o.vx+o.vy*o.vy) << std::endl;
  }
  return objects;
}

}
}
}

#endif // UTILITIES_HPP
