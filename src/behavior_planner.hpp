#ifndef BEHAVIOR_PLANNER_HPP
#define BEHAVIOR_PLANNER_HPP

#include "common_types.hpp"

#include <vector>

class BehaviorPlanner {
public:
  using Object = Common::Object;
  using Pose = Common::Pose;
  using lane_t = Common::lane_t;

  BehaviorPlanner(unsigned int initial_lane);
  unsigned int getDesiredLane(const Pose & current_pose, const std::vector<Object> & objects);
private:
  enum state_t {
    kKeepLane = 0,
    kChangeLeft,
    kChangeRight,
    kPrepareChangeLeft,
    kPrepareChangeRight
  };

  static std::vector<double> getLaneCosts(Pose current_pose, const std::vector<Object> &objects);

  state_t state_{kKeepLane};
  unsigned lane_change_prepare_count_{0};
  lane_t lane_;

};

#endif // BEHAVIOR_PLANNER_HPP
