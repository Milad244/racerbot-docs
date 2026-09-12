# Lecture 10 Notes

**Lecture 10**: https://www.youtube.com/watch?v=x9s8J4ucgO0

## Pure Pursuit

### Overview

* **Pure Pursuit** is a geometric path-tracking algorithm used for lateral vehicle control.
* The vehicle follows a reference path by steering toward a **goal point** located some distance ahead.
* It relies on the **kinematic vehicle model** and assumes **no wheel slip**, ignoring vehicle dynamics.
* It is popular because it is simple and effective for robotics and autonomous driving.

### Autonomous Vehicle Stack

The system follows a **Sense → Plan → Act** loop:

* **Perception:** processes sensor data, builds/uses maps, and localizes the vehicle.
* **Planning:**

  * **Mission planner:** determines the overall goal.
  * **Behavioral planner:** decides what rules/actions to follow.
  * **Local planner:** finds an optimal local trajectory.
* **Control:** tracks the trajectory by generating steering and acceleration commands.

This loop typically runs at **20–50 Hz**.

## Core Idea

1. The vehicle is given a sequence of 2D **waypoints**.
2. It selects a **goal point** approximately a fixed **lookahead distance** $L$ ahead.
3. It computes a circular arc from the vehicle to the goal point.
4. The required curvature determines the steering command.
5. As the vehicle moves, the goal point and steering command are continuously updated.

The vehicle can be viewed as **chasing a moving point** along the path.

## Assumptions

* A sequence of waypoints is available.
* The vehicle can localize itself in the global map.
* Waypoints can be transformed into the vehicle's local coordinate frame.
* The vehicle follows a kinematic bicycle model.
* The vehicle's local frame is centered at the **rear axle**, with:

  * $x$: forward direction
  * $y$: left direction

## Geometric Interpretation

The goal point is represented as:

$$
(x, y)
$$

The distance from the rear axle to the goal point is the **lookahead distance**:

$$
L^2 = x^2 + y^2
$$

To obtain a unique circular arc, its center is constrained to lie on the vehicle's **y-axis**.

The resulting radius is:

$$
R = \frac{L^2}{2|y|}
$$

Therefore, curvature is:

$$
\kappa = \frac{1}{R} = \frac{2|y|}{L^2}
$$

The steering command is chosen to be proportional to this curvature.

## Picking the Goal Point

Ideally, select the point on the path exactly $L$ away from the vehicle.

If no waypoint lies exactly at that distance:

* Find nearby waypoints surrounding the lookahead distance.
* **Interpolate** between them to find the intersection with the lookahead circle.
* Alternatively, use another suitable method for selecting the next waypoint.

### Basic Update Algorithm

1. Determine the current vehicle position.
2. Find the closest point on the path.
3. Find a goal point approximately $L$ ahead.
4. Transform the goal point into vehicle coordinates.
5. Calculate curvature.
6. Convert curvature into a steering command.
7. Update the vehicle position and repeat.

## ROS Pipeline

For each new vehicle pose:

**LiDAR → Particle Filter → Pose → Pure Pursuit → Steering Command**

1. The particle filter estimates the vehicle pose.
2. Pure Pursuit receives the pose.
3. A new goal waypoint is selected.
4. Curvature and steering angle are calculated.
5. The steering command is published.
6. The process repeats whenever a new pose is received.

## Effect of Lookahead Distance $L$

### Small $L$

* More aggressive steering.
* Higher curvature.
* Better ability to follow tight corners.
* Can cause oscillation or swerving due to steering delay.
* May generate dynamically infeasible trajectories.

### Large $L$

* Smoother trajectory.
* Less aggressive steering.
* Larger tracking error.
* May cut corners or get too close to obstacles.

**The lookahead distance is the main tuning parameter of Pure Pursuit.**

## Speed and Lookahead Tuning

* Use **lower speeds in high-curvature regions** such as corners.
* Use **higher speeds on straights**.
* Lookahead distance can depend on vehicle speed:

$$
L \propto v
$$

* Practical implementations may enforce minimum and maximum lookahead distances.

## Limitations

* Pure Pursuit does **not account for vehicle dynamics**.
* It assumes the no-slip condition.
* Aggressive maneuvers can violate this assumption and produce infeasible paths.
* It works best at slower speeds and when tires remain in the **linear tire region**.

## Practical Workflow

1. Create a map using SLAM.
2. Generate or record a set of waypoints.
    - Easiest way is to record waypoints driven by teleop
3. Optionally smooth and evenly space the path.
4. Localize the vehicle using a particle filter.
5. Subscribe to the vehicle pose.
6. Select a lookahead goal point.
7. Compute curvature and steering.
8. Repeat continuously to track the path.

## Key Takeaway

**Pure Pursuit tracks a path by continuously selecting a point ahead of the vehicle and steering along the circular arc that reaches that point.**

Its behavior is primarily controlled by the **lookahead distance $L$**:

* **Smaller $L$** → more accurate but aggressive.
* **Larger $L$** → smoother but less accurate.
