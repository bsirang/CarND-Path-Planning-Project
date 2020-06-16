#include "behavior_planner.hpp"
#include "utilities.hpp"

#include <algorithm>
#include <iostream>

using namespace Common::Utilities;
using namespace Common;

BehaviorPlanner::BehaviorPlanner(unsigned int initial_lane) : lane_{initial_lane} {}


unsigned int BehaviorPlanner::getDesiredLane(const Pose &current_pose, const std::vector<Object> &objects) {
  auto lane_costs = getLaneCosts(current_pose, objects);
  lane_t best_lane = std::min_element(lane_costs.begin(), lane_costs.end()) - lane_costs.begin();
  lane_t chosen_lane = lane_;
  auto old_state = state_;
  auto current_lane = getCurrentLaneNo(current_pose);
  switch (state_) {
  case kKeepLane:
    if (current_lane != lane_) {
      break; // wait until we're in the desired lane before making any decisions
    }
    if (best_lane < lane_) {
      state_ = kPrepareChangeLeft;
    } else if (best_lane > lane_) {
      state_ = kPrepareChangeRight;
    }
    lane_change_prepare_count_ = 0;
    break;
  case kChangeLeft:
    chosen_lane = lane_ - 1;
    state_ = kKeepLane;
    break;
  case kChangeRight:
    chosen_lane = lane_ + 1;
    state_ = kKeepLane;
    break;
  case kPrepareChangeLeft:
    if (best_lane < lane_) {
      ++lane_change_prepare_count_;
    } else if (lane_change_prepare_count_) {
      --lane_change_prepare_count_;
    }
    if (lane_change_prepare_count_ >= kPrepareChangeThreshold) {
      state_ = kChangeLeft;
    } else if (lane_change_prepare_count_ == 0) {
      state_ = kKeepLane;
    }
    break;
  case kPrepareChangeRight:
    if (best_lane > lane_) {
      ++lane_change_prepare_count_;
    } else if (lane_change_prepare_count_) {
      --lane_change_prepare_count_;
    }
    if (lane_change_prepare_count_ >= kPrepareChangeThreshold) {
      state_ = kChangeRight;
    } else if (lane_change_prepare_count_ == 0) {
      state_ = kKeepLane;
    }
    break;
  }
  if (state_ != old_state) {
    std::cout << "STATE CHANGE " << old_state << " -> " << state_ << std::endl;
  }
  if (lane_ != chosen_lane) {
    std::cout << "New desired lane " << chosen_lane << std::endl;
    lane_ = chosen_lane;
  }
  std::cout <<  lane_costs[0] << "\t\t" << lane_costs[1] << "\t\t" << lane_costs[2] << std::endl;

  return lane_;
}


std::vector<double> BehaviorPlanner::getLaneCosts(Pose current_pose, const std::vector<Object> &objects) {
  std::vector<double> lane_costs{0.0, 0.0, 0.0};
  auto current_lane = getCurrentLaneNo(current_pose);
  current_pose.s += (current_pose.speed * kProjectAheadTime);

  // We can rely on our proximity detector to slow us down as needed to not
  // hit the car in front of us. Given that, we can simply use our current
  // speed as an indicator of how "costly" it is to stay in our lane
  double stay_in_lane_cost = std::max(kMaxVelocity - current_pose.speed, 0.0) / kMaxVelocity;
  lane_costs[current_lane] = stay_in_lane_cost;

  for (const auto & object : objects) {
    Pose p{object};
    // Project the object ahead
    p.s += p.speed * kProjectAheadTime;
    auto object_lane = getCurrentLaneNo(p);
    bool adjacent_lane = (::abs(current_lane - object_lane) == 1);
    bool same_lane = (current_lane == object_lane);
    if (adjacent_lane) { // adjacent lane
      double delta = ::fabs(p.s - current_pose.s);
      double cost = 0.0;
      if (delta <= kOccupancyWindow) {
        // We consider this lane occupied, so it is costly to change lanes into it
        cost = ((1.0 - (delta / kOccupancyWindow)) * (1.0 - kLaneOccupiedBaseCost)) + kLaneOccupiedBaseCost;
      } else if (p.s > current_pose.s && (p.s - current_pose.s) < kObjectAheadThreshold) {
        // So the object is ahead of us, and by more than the occupancy window, but
        // still close enough that we will consider the vehicles speed
        cost = ((std::max(kMaxVelocity - p.speed, 0.0) / kMaxVelocity) * (1.0 - kLaneChangeBaseCost)) + kLaneChangeBaseCost;
      } else {
        cost = kLaneChangeBaseCost;
      }
      if (cost > lane_costs[object_lane]) {
        lane_costs[object_lane] = cost;
      }
    } else if (!same_lane) {
      // we can't consider objects in lanes that are not adjacent
      lane_costs[object_lane] = 2.0; // too high to ever pick
    }
  }
  return lane_costs;
}
