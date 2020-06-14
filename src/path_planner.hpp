#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <utility>
#include <vector>

class PathPlanner {
public:
  struct Pose {
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
  };
  using points_2d_t = std::pair<std::vector<double>, std::vector<double>>;
  using points_1d_t = std::vector<double>;
  using point_t = std::pair<double, double>;
  using sensor_fusion_t = std::vector<std::vector<double>>;

  PathPlanner(const points_2d_t & map_xy, const points_2d_t & map_dxdy, const points_1d_t & map_s, double delta_t);

  points_2d_t getNextPath(const points_2d_t & previous_path, const point_t & path_end, const Pose & current_pose, sensor_fusion_t fusion_results);

private: // constants
  static constexpr double kMphToMs = 2.237;
  static constexpr unsigned kNumPoints = 50;
  static constexpr double kMaxVelocity = 49.5 / kMphToMs;
  static constexpr double kMaxAcceleration = 10; // m/s^2
  static constexpr double kMaxJerk = 10; // m/s^3

private: // methods
  point_t generatePointAheadFrenet(point_t point, double velocity);
  static double updateKinematics(double current, double target, double limit, double delta_t);

private: // members
  const points_2d_t map_xy_;
  const points_2d_t map_dxdy_;
  const points_1d_t map_s_;
  const double delta_t_;

  double target_velocity_{kMaxVelocity};
  double current_velocity_{0.0};

  double target_acceleration_{kMaxAcceleration};
  double current_acceleration_{0.0};
};

#endif // PATH_PLANNER_HPP
