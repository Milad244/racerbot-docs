# Lecture 9 Notes

## Introduction to Graph-based SLAM

Previously, with particle filters, we were **given a map**. Now we are not.

Three related problems:
- **Localization:** estimate the robot's poses given landmarks
- **Mapping:** estimate the landmarks given the robot's poses
- **SLAM:** estimate the robot's poses *and* the landmarks at the same time

![SLAM Example](/assets/module-c/lecture-9/slam-example.png)

Two families of approaches to SLAM:

| | **Filtering** | **Smoothing** |
|---|---|---|
| Methods | EKF, Particle Filter | Pose Graph Optimization |
| Approach | Online state estimation as new measurements become available | Full trajectories estimated using the complete set of measurements |

![Filtering vs. Smoothing](/assets/module-c/lecture-9/filtering-vs-smoothing.png)

## Problem Setting: SLAM

The Simultaneous Localization and Mapping problem asks whether it is possible for a mobile robot placed at an **unknown location** in an **unknown environment** to incrementally build a consistent map of that environment while simultaneously determining its location within this map.

The conceptual breakthrough came with the realization that the combined mapping and localization problem, once formulated as a **single estimation problem**, was actually convergent.

Correlations between landmarks, which most researchers had tried to minimize, were actually the critical part of the problem. On the contrary, the more these correlations grew, the better the solution.

## Idea of Graph-Based SLAM

- Use a **graph** to represent the problem
- Every **node** in the graph corresponds to a pose of the robot during mapping (and its laser measurement)
- Every **edge** between two nodes corresponds to a spatial constraint between them
- Graph-Based SLAM: build the graph and find a node configuration that **minimizes the error** introduced by the constraints

In a nutshell:
1. Every node = a robot position + a laser measurement
2. An edge between two nodes = a spatial constraint between them
3. Once we have the graph, determine the most likely map by **correcting the nodes**
4. Then render a map based on the known poses

![Graph-Based SLAM in a Nutshell](/assets/module-c/lecture-9/graph-based-slam-nutshell.png)

![Graph SLAM Corrected Map](/assets/module-c/lecture-9/graph-slam-corrected-map.png)

### The Overall SLAM System
An interplay of front-end and back-end:

**raw data → Graph Construction (Front-End) → graph (nodes & edges) → Graph Optimization (Back-End) → node positions**

The map helps determine constraints by reducing the search space. The topic of this lecture is the **optimization** (back-end).

![SLAM Front-End and Back-End](/assets/module-c/lecture-9/slam-front-end-back-end.png)

## A Toy Problem: Dead Reckoning with F1TENTH

- Calculating the current position of a moving robot based on a previously determined position
- In our case, the VESC determines the current ERPM of the motor and converts it into the vehicle's longitudinal velocity. The servo angle is used to determine the current steering angle
- Using a basic kinematics model, the relative position of the car can be determined. This is **odometry**

### Sensing the Environment
Put the car in an unknown environment; it has no idea what is around it. The LiDAR gives range measurements that represent part of the environment.

### In an Ideal World
With no uncertainty or errors in the LiDAR or odometry measurements:
- If odometry is perfectly accurate, and
- if the LiDAR range measurements are perfectly accurate,
- then we can use the relative movements to translate and rotate the sensor measurements accordingly, and piece together a map from them

After a while we have built up a representation of the environment. Run the robot around longer, gather more measurements, and it becomes a true representation of the environment.

![Ideal World Map](/assets/module-c/lecture-9/ideal-world-map.png)

### But in the Real World
This scenario is not realistic. There is error in both the LiDAR measurement and the odometry, so there is uncertainty in the estimated robot pose and in the distances to measured obstacles.

- Odometry is not accurate (wheel slip, inaccuracy in measuring motor RPM, etc.)
- The LiDAR scan, although pretty accurate, has noise in each measurement
- This means our perfect-scenario map-making algorithm **breaks**

![Real World Error](/assets/module-c/lecture-9/real-world-error.png)

Walking through the failure:
1. We get an initial LiDAR measurement and assume there is an obstacle there, just like before
2. We drive a bit, except this time the **estimated pose differs from the real pose**
3. The LiDAR returns correct relative distances to the wall the real robot sees, but we can only assume they are relative to the *estimated* pose, since that is all we know
4. The mapping algorithm mistakenly puts the points in the wrong position using the wrong odometry
5. This places the two measured obstacles in **different frames** rather than a global environment frame, so they do not line up
6. Continuing around the room, the odometry error causes the estimated pose to deviate further and further from the real pose

![Real World Drift](/assets/module-c/lecture-9/real-world-drift.png)

The end result is a map of obstacles that does not resemble the real environment at all. Uncertainty in our system messes it up, and this is why we cannot just take measurements and stick the results into a map.

![Real World Final Map](/assets/module-c/lecture-9/real-world-final-map.png)

**This is where SLAM algorithms help.**

## SLAM Overview: Two Approaches

### Filtering-based State Estimation
- Use EKF or a Particle Filter for state estimation and build the map based on that estimation
- Examples: **GMapping**, **HectorSLAM**
- EKF or particle filters improve the state estimation and provide better transformations between measurements
- **However:** since filtering still cannot provide the ground-truth position of the car, the compounding error still exists. This approach struggles when mapping large environments, or when the dead reckoning is so far off that filtering cannot pull it back to correct estimations

### Pose Graph Optimization (Graph-based)
- Use **pose graphs** to store constraints between pose estimations
- Use optimization for **loop closure** to correct pose estimations
- Examples: **KartoSLAM**, **Cartographer**. We will be using `slam_toolbox`, also graph-based

How it works:
1. Store pose estimations and measurements in a pose graph, keeping the relative pose estimations as constraints **with confidence**
2. A sensor matcher (a scan matcher, in our 2D LiDAR case) finds correspondence between nodes in the graph
3. The constraints in the pose graph are adjusted **all at once** as an optimization problem, which corrects the estimation errors

## Pose Graphs: Edge Constraints

The first thing the robot does is take a measurement of the environment. That measurement is associated with the current estimated robot pose, and both go into the pose graph. So a pose is defined as an X and Y location and a rotation angle, saved along with the distances and angles to the sensed obstacles. There are also **uncertainties** associated with this pose entry.

The robot drives a little, its estimated pose starts to deviate from the real pose, takes another measurement, and that combination is saved as the second entry.

### The Spring Analogy
There is a constraint on the relative distance between two poses. Ideally they stay exactly that far apart since that is our best estimate, but due to uncertainty in the odometry process, we might be better off moving these two poses relative to each other.

To visualize a constraint, picture a **spring or rubber bar** connecting the two poses:
- The **nominal length** of the bar is how far apart we estimate them to be
- With no external forces, the bar keeps them at this fixed distance
- If you hold one pose fixed and move the other closer or further, it compresses or stretches the bar, creating a **restoring force**

![Edge Constraint Spring](/assets/module-c/lecture-9/edge-constraint-spring.png)

### Stiffness = Confidence
The strength of the rubber bar depends on **how confident we are in the distance estimate**:
- More confidence (a really good odometry process) → a **strong, stiff** bar, hard to change the nominal distance
- Almost no confidence → a **weak, soft** bar providing almost no restoring force

Since this edge is created from odometry: the more confident you are in your odometry measurement, the stiffer the spring and the more resistant to change.

![Edge Constraint Stiffness](/assets/module-c/lecture-9/edge-constraint-stiffness.png)

The point is that we are creating the **edges** of the graph, modeling "tension" between nodes, so the nodes can be moved around to fit constraints we define later.

> This is exactly the scan matching problem we have seen before.

### Nomenclature
- The **poses are the nodes** of the graph
- The **constraint (the rubber bar) is an edge**: the relative pose estimation between nodes, obtained from odometry in our case
- The constraint acts in all **three pose dimensions**: X, Y, and rotation, always trying to bring the nodes back to their estimated distance

Building the graph one pose at a time gives us something that looks like the incorrect map we built earlier, with the exception that there are now **constraints connecting all of the poses**. At this point all the bars are at their nominal length and everything wants to stay right where it is, so we still cannot do much with this information alone.

## Pose Graph Optimization: Loop Closure

Something interesting happens when the **first pose and the current pose observe the same feature** in the environment.

This means we can build a **new edge**, a new constraint between these two nodes. We just need to understand how the two features align to figure out where the two poses have to be relative to each other. In this example they would have to be in the exact same location, so this new spring has a **nominal length of 0**.

![Loop Closure: Observing the Same Feature](/assets/module-c/lecture-9/loop-closure-same-feature.png)

The **stiffness** of this spring is determined by your confidence in your sensor measurement and scan matcher. If you are really confident the two measurements should be the same, the spring pulls the two nodes together very hard.

### Reaching Equilibrium
- The new (blue) bar wants to pull the two nodes together
- All the other (purple) bars want to keep their relative distances as they are
- We have injected **tension** into the edges of the pose graph

![Loop Closure Tension](/assets/module-c/lecture-9/loop-closure-tension.png)

Allowing this graph to **settle to equilibrium**, balancing all the forces the constraints impose, is the *optimization* part of pose graph optimization. This is **loop closure**: we solve for the tensions between each node to reach equilibrium, as an optimization problem.

When the graph reaches equilibrium, the collected data looks a lot better and resembles the true environment.

![Loop Closure Equilibrium](/assets/module-c/lecture-9/loop-closure-equilibrium.png)

### Why Loop Closure Is So Valuable
By optimizing the pose graph, we get not only a better estimate of the **current** pose and a better model of the environment, but also a better estimate of **where the robot was in the past**, since all past poses are updated too. We get a lot of value from one loop closure.

This is the essential step in graph-based SLAM. It is what makes graph SLAM work.

### More Loop Closures
- Before a new loop closure, the graph is in equilibrium, so no further adjustments are made
- After adding new constraints for loop closure, the graph is **no longer in equilibrium**
- The same optimization runs again, pulling on other springs a bit and resetting the graph into equilibrium
- We can continue to improve our estimates by making **more** loop closures

### Failure Modes
- If you are **falsely confident** in making an association between observations and make a **wrong loop closure**, the optimization ends up in the **wrong equilibrium**
- In fact, it is better to **miss** an actual loop closure than to make a wrong one
- It is also important to use **absolute measurements** of the environment in the loop closure step. If you use a relative measurement like the IMU, you cannot really be sure whether a current state matches a past one

## Storing the Data as a Map

In our 2D case we use **occupancy grids**.

### Binary Occupancy Grid
Fill in the grids that have a data point in them as occupied. Problem: if a grid is **mistakenly hit**, whether from localization errors or LiDAR noise, it stays an occupied grid forever.

### Probabilistic Occupancy Grid
Each grid stores the **probability** that it is occupied. The odds in the grid are continuously updated according to "hits" and "misses" by the laser scan. This is a better solution than the binary grid:

If there are false hits, it is unlikely they happen more than once in the exact same way. The way a probabilistic grid is constantly updated will eventually **remove the false positive hit**.

![Binary vs. Probabilistic Occupancy Grid](/assets/module-c/lecture-9/occupancy-grid-binary-vs-probabilistic.png)

### Thresholding
To go from the probabilistic grid to a usable map:

$$
p(x,y) \le 0.5 \;\rightarrow\; 1, \qquad p(x,y) > 0.5 \;\rightarrow\; 0
$$

![Probabilistic Sensor Model](/assets/module-c/lecture-9/probabilistic-sensor-model.png)

## Final Notes on Graph-based SLAM

Graph-based SLAM is **sensor agnostic**. It only requires 2 types of sensors:
1. One that can measure **relative transformations** between poses (IMU, odometry, wheel encoders, etc.), used to build constraints between nodes
2. One that can measure **absolute observation** of the environment (LiDARs, cameras, etc.), used for loop closure

The front end and the back end are executed in an **interleaved** way. There are many more caveats in realizing such a system in software.

## Definition of the SLAM Problem

**Given:**
- The robot's controls
- Observations

**Wanted:**
- Map of the environment
- Path of the robot

### Probabilistic Approaches
There is uncertainty in the robot's motions and observations, so we use probability theory to explicitly represent that uncertainty. It is the difference between "the robot is exactly here" and "the robot is somewhere here."

### In the Probabilistic World
Estimate the robot's path and the map:

$$
p(x_{0:T}, m \mid z_{1:T}, u_{1:T})
$$

- $p(\cdot)$: distribution
- $x_{0:T}$: path
- $m$: map
- $z_{1:T}$: observations (given)
- $u_{1:T}$: controls (given)

![SLAM Probabilistic Formulation](/assets/module-c/lecture-9/slam-probabilistic-formulation.png)

In the graphical model, the **path and map are unknown** while the observations and controls are **observed**.

### Full SLAM vs. Online SLAM
**Full SLAM** estimates the entire path:

$$
p(x_{0:T}, m \mid z_{1:T}, u_{1:T})
$$

**Online SLAM** seeks to recover only the most recent pose:

$$
p(x_t, m \mid z_{1:t}, u_{1:t})
$$

![Full SLAM vs. Online SLAM](/assets/module-c/lecture-9/full-vs-online-slam.png)

Online SLAM means **marginalizing out the previous poses**:

$$
p(x_t, m \mid z_{1:t}, u_{1:t}) = \int \dots \int p(x_{0:t}, m \mid z_{1:t}, u_{1:t})\, dx_{t-1} \dots dx_0
$$

These integrals are typically solved **recursively**, one at a time.

## Why is SLAM a Hard Problem?

### 1. Robot Path and Map Are Both Unknown
Map and pose estimates are **correlated**. The position estimate of the newest landmarks depends on the robot position, which in turn depends on the observed landmark the robot used to correct its position, so landmarks become correlated with each other.

![SLAM is Hard: Correlated Estimates](/assets/module-c/lecture-9/slam-hard-correlated.png)

### 2. Known vs. Unknown Correspondence
We cannot uniquely identify landmarks, and wrong data associations lead to problems in state estimation.

The **correspondence problem** is the problem of relating the identity of sensed things to other sensed things. Some SLAM algorithms assume landmark identity is known; others provide special mechanisms for estimating the correspondence of measured features to previously observed landmarks in the map. Estimating this correspondence is known as the **data association problem**, and it is one of the most difficult problems in SLAM.

- The mapping between observations and the map is unknown
- Picking wrong data associations can have **catastrophic consequences (divergence)**

![SLAM is Hard: Data Association](/assets/module-c/lecture-9/slam-hard-data-association.png)

### Three Traditional Paradigms
**Kalman filter | Particle filter | Graph-based**

Graph-based is the **least squares approach** to SLAM.

![Three Traditional Paradigms](/assets/module-c/lecture-9/three-paradigms.png)

## Motion and Observation Models

### Motion Model
Describes the relative motion of the robot:

$$
p(x_t \mid x_{t-1}, u_t)
$$

- distribution over the **new pose**, given the **old pose** and the **control**
- Can be a Gaussian or a non-Gaussian model

### Observation Model
Relates measurements with the robot's pose:

$$
p(z_t \mid x_t)
$$

- distribution over the **observation**, given the **pose**
- Can be a Gaussian or a non-Gaussian model

### Model for Virtual Observations
Relate pairs of poses from which observations have been recorded. This gives us knowledge about the **relative poses**.

## Least Squares SLAM

### Least Squares in General
- An approach for computing a solution for an **overdetermined** system
- "More equations than unknowns"
- Minimizes the sum of the **squared errors** in the equations
- A standard approach to a large set of problems

### The Graph
- It consists of $n$ nodes $\mathbf{x} = \mathbf{x}_{1:n}$
- Each $\mathbf{x}_i$ is a pose of the robot at time $t_i$
- A constraint/edge exists between the nodes $\mathbf{x}_i$ and $\mathbf{x}_j$ if...

**Create an edge if (1):** the robot moves from $\mathbf{x}_i$ to $\mathbf{x}_{i+1}$ → the edge corresponds to **odometry**, representing the odometry measurement.

**Create an edge if (2):** the robot observes the same part of the environment from $\mathbf{x}_i$ and from $\mathbf{x}_j$ → construct a **virtual measurement** about the position of $\mathbf{x}_j$ seen from $\mathbf{x}_i$. The edge represents the position of $\mathbf{x}_j$ seen from $\mathbf{x}_i$ based on the observation.

![Create an Edge from Observation](/assets/module-c/lecture-9/create-edge-observation.png)

### Transformations
Transformations can be expressed using **homogeneous coordinates**:
- **Odometry-based edge:** $(\mathbf{X}_i^{-1}\mathbf{X}_{i+1})$
- **Observation-based edge:** $(\mathbf{X}_i^{-1}\mathbf{X}_j)$, how node $i$ sees node $j$

### Homogeneous Coordinates
- A system of coordinates used in **projective geometry**, an alternative representation of geometric objects and transformations
- A single matrix can represent both **affine** and **projective** transformations
- An N-dimensional space is expressed in N+1 dimensions (4 dimensions for modeling 3D space)

### The Edge Information Matrices
- Observations are affected by noise
- An **information matrix** $\boldsymbol{\Omega}_{ij}$ for each edge encodes its uncertainty
- The "bigger" $\boldsymbol{\Omega}_{ij}$, the more the edge "matters" in the optimization

Questions worth thinking about:
- How do the information matrices look in the case of scan matching vs. odometry?
- How will these matrices look when moving down a long, featureless corridor?

### The Pose Graph
For an edge between nodes $\mathbf{x}_i$ and $\mathbf{x}_j$:
- $\langle \mathbf{z}_{ij}, \boldsymbol{\Omega}_{ij} \rangle$: the edge, the observation of $\mathbf{x}_j$ from $\mathbf{x}_i$, with its information matrix
- $\mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)$: the **error** between the nodes according to the graph and the observation

**Goal:**

$$
\mathbf{x}^{*} = \arg\min_{\mathbf{x}} \sum_{ij} \mathbf{e}_{ij}^{T}\, \boldsymbol{\Omega}_{ij}\, \mathbf{e}_{ij}
$$

![Pose Graph Error](/assets/module-c/lecture-9/pose-graph-error.png)

This error function is exactly the form suited to **least squares error minimization**.

## Sparse Pose Adjustment

The task of pose-graph optimization is to seek a configuration of the nodes that **maximizes the likelihood of the measurements** encoded in the constraints. We use **Sparse Pose Adjustment (SPA)** to find the graph configuration.

### Measurement Equation
For robot poses $c_i$ and $c_j$, the measurement equation (constraint, or offset) is:

$$
h(c_i, c_j) \equiv \begin{cases} R_i^{\top}(t_j - t_i) \\ \theta_j - \theta_i \end{cases}
$$

- $R_i^{\top}$: rotation matrix of $\theta_i$
- $t_j - t_i$: translation
- $\theta_j - \theta_i$: difference in angle

### Error Function

$$
e_{ij} \equiv \bar{z}_{ij} - h(c_i, c_j)
$$

- $\bar{z}_{ij}$: the measured offset between $c_i$ and $c_j$, from scan matching, with a **precision matrix** (the inverse of covariance)

### Total Error

$$
\chi^2(\mathbf{c}, \mathbf{p}) \equiv \sum_{ij} e_{ij}^{\top}\, \Lambda_{ij}\, e_{ij}
$$

- $\Lambda_{ij}$: precision matrix

The optimal placement of each $c$ is found by **minimizing the total error**. This is a least squares problem, solved using the **Levenberg-Marquardt (LM)** framework.

![Sparse Pose Adjustment Total Error](/assets/module-c/lecture-9/sparse-pose-adjustment-total-error.png)

## Scan Matching Methods

Graph-based SLAM is also **scan (sensor) matching method agnostic**:
- ICP (what we learned in the scan matching lecture)
- Scan-to-scan, Scan-to-map, Map-to-map
- Feature-based
- RANSAC for outlier rejection
- Bundle Adjustment
- Correlative Scan Matching

### Correlative Scan Matching
- PL-ICP/ICP explicitly computes correspondence between two scans for a transformation
  - Susceptible to **local minima**; poor initial estimates lead to incorrect data associations
- **Correlative Scan Matching** searches for transformations *without* computing correspondences, projecting one set of scans on top of a reference map
- The reference map is generally implemented as a **look-up table** assigning a cost for every projected query point

### Probabilistic View of Scan Matching
Previously we viewed scan matching as finding an absolute solution. Now we take a Bayesian point of view.

Say the robot moves from $x_{i-1}$ to $x_i$ with some motion $u$ and observation $z$, depending on the environment model $m$ and the robot's position. Our goal is the posterior $p(x_i \mid x_{i-1}, u, m, z)$. With Bayes' rule:

$$
p(x_i \mid x_{i-1}, u, m, z) \propto p(z \mid x_i, m)\, p(x_i \mid x_{i-1}, u)
$$

On the RHS, the first term is the **observation model** and the second is the **motion model**. The motion model is easy to guess (multivariate Gaussian, estimated with our odometry), so our goal is to approximate the observation model.

Assume individual scan points are **independently observed**:

$$
p(z \mid x_i, m) = \Pi_j\, p(z_j \mid x_i, m)
$$

For 2D LiDAR scans, each of these distributions is based on which surface of the map $m$ would be visible at $x_i$. We could use **ray marching** to find this, but it is expensive (we do use ray marching in our particle filter on the GPU for the same observation model). Correlative Scan Matching instead uses a **2D look-up table** containing the log probabilities of a LiDAR observation at each $(x, y)$ position in the world.

### Scoring Candidate Transformations
- Candidate transformations are scored according to a **cost map**
- Rendered into the cost map with a **blurring kernel** to approximate uncertainty in scans
- Candidates are generated from plausible priors (e.g. from odometry)
- The **covariance** of the solution can also be principally calculated (this matters, since it becomes the edge information matrix)

![Correlative Scan Matching Cost Map](/assets/module-c/lecture-9/correlative-scan-matching-costmap.png)

The optimal transformation is:

$$
T^{*} = \arg\max_{T} \sum_{i \in N} L(T p_i)
$$

- $T^{*}$: optimal transformation
- $\arg\max_T$: over all plausible transformations
- $\sum_{i \in N}$: sum over all scan points
- $L(\cdot)$: lookup function
- $T p_i$: points transformed into a common frame of reference

![Correlative Scan Matching Formula](/assets/module-c/lecture-9/correlative-scan-matching-formula.png)

## Summary
- **Mapping** is the task of modeling the environment
- **Localization** means estimating the robot's pose
- **SLAM** = simultaneous localization and mapping
- Full SLAM vs. Online SLAM

## What's Next?
- A natural step after creating a map is to localize the robot within a known map. We have done that already with the **Particle Filter**
- With the ability to make a map and localize, we move on to **motion planning** in the next few lectures
- We will also go over how to run `slam_toolbox` on the cars

## Key Takeaways
- Filtering-based SLAM (EKF, particle filter) still accumulates compounding error, because filtering never recovers ground truth. It struggles on large environments and badly-off dead reckoning
- Graph-based SLAM reframes the problem: **nodes are poses + measurements**, **edges are spatial constraints**, and mapping becomes finding the node configuration that minimizes constraint error
- The spring analogy: each edge is a rubber bar whose nominal length is the estimated relative pose and whose **stiffness is your confidence** in that estimate
- **Loop closure** is what makes graph SLAM work. Recognizing a revisited place adds a constraint that, when the graph re-settles, corrects not just the current pose but the **entire past trajectory**
- A wrong loop closure lands the optimization in the wrong equilibrium. It is better to miss a loop closure than to make a false one, and loop closure needs **absolute** environment measurements, not relative ones like IMU
- Probabilistic occupancy grids beat binary ones because constant hit/miss updating eventually washes out false positives
- Formally: minimize $\sum_{ij} \mathbf{e}_{ij}^T \boldsymbol{\Omega}_{ij} \mathbf{e}_{ij}$, an overdetermined least squares problem solved via Sparse Pose Adjustment with Levenberg-Marquardt
- Graph-based SLAM is **sensor and scan-matcher agnostic**. It only needs one sensor for relative transformations and one for absolute environment observation
