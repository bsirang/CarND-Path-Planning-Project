# Highway Driving Path Planner Project

## Goal
The goal of this project is to implement a functional path planner that can navigate around other vehicles on a freeway. A simulator is used to feed the path planner information about your vehicle's pose as well as the other surrounding vehicles.

The output of the path planner is a series of points in 2D, which the simulator will follow at 50 Hz. The vehicle's control ability is "perfect" in the sense that the simulator will follow the points at 50Hz no matter what. This ensures that the simulator's control response won't mask trajectories which are not smooth.

The vehicle must not exceed the following:
* Accelerations (including tangential) of 10 m/s^2.
* Jerks of 10 m/s^3
* Forward velocity of 50 mph

## Approach
The approach involves smooth trajectory generation, proximity detection, and behavior planning state machines and cost functions.

### Smooth Trajectory Generation
In order to guarantee that acceleration and jerk limits are met, kinematic equations were used to limit the change in acceleration and velocity over the 0.02 second time interval. This way the target velocity (which may instantaneously change without bounds) will move the current velocity along a smooth curve.

#### Splines
Splines were also calculated to enable the vehicle to turn smoothly around curves. Without splines, the discontinuities or "kinks" in the curvature of the road would cause spikes in acceleration/jerk.

In order to determine the spacing of points along the spline to maintain velocity, a triangular approximation was used. The approach was to calculate the euclidean distance to a point some arbitrary distance ahead on the spline, dividing that distance into intervals based on the desired speed, projecting that distance along the vehicle's forward axis, and then calculating points along the spline at those intervals.

### Proximity Detection
In order to avoid rear ending vehicles in front of us, a proximity detector was implemented, which adjusts the target velocity such that the worst-case braking distance (as a function of our current velocity and deceleration curve) is maintained. This will allow us to stop in time if the vehicle in front decides to slam on their brakes.

### Changing Lanes
Now that the path planner can generate a smooth trajectory that slows the vehicle down in order to avoid hitting vehicles in front, we need to decide when to switch lanes.

When deciding to change lanes, our path planner must do it gracefully, adjusting to the speed of traffic in the new lane.

#### Finite State Machine
Given the constrained environment of a freeway, we can break down our behavior into a handful of discrete states. As suggested by the course work the following states were chosen:
* Keep Lane
* Consider Lane Change Left
* Consider Lane Change Right
* Change Lane Left
* Change Lane Right

The transitions between these states are determined by a cost function.

#### Cost Function
The job of the cost function is to decide which lane is best to be in for any given moment in time. The cost function as implemented in this project represents the following logic as written out in plain english:

* Even if the lane adjacent to me is traveling ever so slightly faster, I shouldn't immediately change lanes because there is some burden to change lanes to begin with.
* If I am going the speed limit in my current lane, I have no incentive to change lanes.
* If I am going slower than the speed limit, I should consider changing lanes.
* I shouldn't change lanes if there is a vehicle too close in the neighboring lane.
* I should analyze the speed of traffic in the adjacent lanes, and change lanes only if the relative speed of the other lane is great enough to be "worth it".

## Shortcomings and Potential Improvements
While there are some shortcomings identified, they haven't actually caused issues in the simulation environment and don't affect the ability to meet the requirements of this project.

### All or nothing lane change
After the planner decides to change lanes, it is fully committed and doesn't react to any potential sudden changes to vehicles in the existing lane. This can be remedied by a more robust proximity detector and additional edges in the state machine (such as a transition from a change lane state back to a keep lane state).

### Cost Function Edge Cases
The way the cost function is implemented, there are some edge cases that could yield undesirable behavior. For example the planner may choose to change lanes if the vehicle comes to a complete stop and the neighboring lane is currently occupied. This can be remedied by changing the cost function logic and testing edge cases thoroughly.

### Reactive lane change decision making
For simplicity of implementation, the decision to change lanes is dictated largely by the current speed of the vehicle, which is largely dictated by the proximity detector. In practice this means the vehicle will generally change lanes after it had slowed down rather than before. This can be remedied by introducing additional inputs into the cost function.

### Treating vehicles as a singular point
Vehicles are treated as a singular point, which means they always occupy a single lane. This means that we don't consider a vehicle to be in our lane until the center point crosses over the lane boundary. This can be remedied by taking vehicle dimensions into account, and considering that a vehicle may be occupying more than one lane at a time.
