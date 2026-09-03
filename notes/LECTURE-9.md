# Lecture 9 Notes

## Introduction to Graph-based SLAM

Lesson plan:
1. Two Approaches to SLAM: Filtering vs. Smoothing
2. Graph-Based SLAM: Nodes, Edges, and the Spring Analogy
3. Loop Closure and Pose Graph Optimization
4. Formalizing SLAM: Full vs. Online SLAM, Why It's Hard
5. Least Squares SLAM: Pose Graphs, Information Matrices, Sparse Pose Adjustment
6. Scan Matching Methods (ICP, Correlative Scan Matching)

Two families of approaches to SLAM:

| | **Filtering** | **Smoothing** |
|---|---|---|
| Methods | EKF, Particle Filter | Pose Graph Optimization |
| Approach | Online state estimation as new measurements become available | Full trajectories estimated using the complete set of measurements |

## Problem Setting: SLAM

**SLAM (Simultaneous Localization and Mapping)** is the problem of a robot in an **unknown environment** determining **where it is** while **building a map** at the same time.

The key insight was to treat localization and mapping as a **single estimation problem**, meaning we estimate the robot's poses and the map together. This problem was shown to be **convergent**, meaning that with enough measurements, the estimates can settle toward a consistent solution.

The robot's pose and map are **correlated** because measurements connect them. For example, if a LiDAR observation tells us something about the environment from a particular pose, changing that pose also changes where we believe the observed feature is. These correlations allow information to propagate through the system and help produce a more consistent estimate.

## Idea of Graph-Based SLAM

- Use a **graph** to represent the problem
- Every **node** corresponds to a robot pose during mapping, along with its laser measurement
- Every **edge** between two nodes represents a spatial constraint, meaning a measurement of how those two poses should be positioned relative to each other
- Graph-Based SLAM builds the graph and finds the node configuration that **minimizes the error** in these constraints

In a nutshell:
1. Every node = a robot position + a laser measurement
2. An edge between two nodes = a spatial constraint between the poses, such as an odometry constraint or loop closure constraint
3. Optimize the graph to **correct the estimated poses** so they best satisfy all constraints
4. Render the map using the corrected poses

![Graph-Based SLAM in a Nutshell](/assets/module-c/lecture-9/graph-based-slam-nutshell.png)

### The Overall SLAM System
An interplay of front-end and back-end:

**raw data → Graph Construction (Front-End) → graph (nodes & edges) → Graph Optimization (Back-End) → node positions**

The map helps determine constraints by reducing the search space. The topic of this lecture is the **optimization** (back-end).

![SLAM Front-End and Back-End](/assets/module-c/lecture-9/slam-front-end-back-end.png)

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

#### How it works:

1. Store pose estimations and measurements in a **pose graph**, keeping the relative pose estimations as **constraints with confidence**
   - For example, odometry might tell us that the robot moved 2 m forward from pose A to pose B.
   - This becomes a constraint describing how A and B should be related, along with how much we trust that measurement.

2. A **sensor matcher** (a scan matcher, in our 2D LiDAR case) finds **correspondences between nodes** in the graph
   - It compares LiDAR scans to determine whether different poses observed the same environment.
   - This can create additional constraints, such as **loop closures**.

3. The constraints in the pose graph are adjusted **all at once** as an optimization problem, which corrects the estimation errors
   - The optimizer adjusts the poses to find the configuration that best satisfies all the constraints together.
   - This corrects errors such as accumulated **odometry drift**.

#### What is stored in the graph:

* **Nodes:** store the robot's **pose estimates** and associated **LiDAR measurements/scans**.

* **Edges:** store **relative pose constraints** between nodes. Different measurements can create different types of edges:

  * **Odometry constraint:** connects consecutive poses and represents how the robot moved from one pose to the next.
  * **Loop closure constraint:** connects two poses that are not necessarily consecutive when the sensor matcher recognizes that the robot has returned to a previously visited location.
  * Each edge also stores the **confidence** in its constraint.

* During optimization, the **nodes are adjusted** so that they best satisfy **all of these constraints together**, correcting errors such as **odometry drift**.

## Pose Graphs: Edge Constraints

The robot takes a measurement and creates a **node** containing its **estimated pose** $(x, y, \theta)$ and **LiDAR scan**. The robot's movement between poses creates an **edge**, containing the **relative pose constraint** and its **uncertainty/confidence**.

After moving, the robot creates another node, and this process continues as the robot builds the pose graph.

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

The point is that we are creating the **edges** of the graph, modeling "tension" between nodes, so the nodes can be moved around to fit constraints we define later.

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

Allowing this graph to **settle to equilibrium**, balancing all the forces the constraints impose, is the *optimization* part of pose graph optimization. A **loop closure** provides an additional constraint when the robot recognizes a previously visited location. The optimizer then uses this new constraint, along with all the others, to adjust the poses and reach equilibrium.

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
- It is also important to use **absolute measurements** of the environment, such as **LiDAR scans**, in the loop closure step. If you use only a relative measurement like the IMU, you cannot really be sure whether a current state matches a past one.

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

- The mapping between sensor observations and landmarks in the map is unknown
- Picking wrong data associations can have **catastrophic consequences (divergence)**

> **Intuition:** Imagine driving through a building with many identical-looking hallways. You see a doorway and need to decide whether it is the **same doorway you saw before** or a different one. If you associate it with the wrong landmark, you create a **wrong constraint**, which can cause the estimated pose and map to become seriously distorted.


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
- An approach for computing a solution for an **overdetermined** (more equations than unknowns) system
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
Transformations can be expressed using **homogeneous coordinates**. Here, $\mathbf{x}_i$ is the robot's **pose**, and $\mathbf{X}_i$ is the **transformation matrix representing that pose**, which lets us mathematically combine poses and calculate the relative transformation between them:


* **Odometry-based edge:** $(\mathbf{X}_i^{-1}\mathbf{X}_{i+1})$ → relative transformation from pose $i$ to pose $i+1$

* **Observation-based edge:** $(\mathbf{X}_i^{-1}\mathbf{X}_j)$ → relative transformation from pose $i$ to pose $j$, based on observing the same environment

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

* **Edge:**

$$
\langle \mathbf{z}_{ij}, \boldsymbol{\Omega}_{ij} \rangle
$$


  $\mathbf{z}_{ij}$ is the **measured** relative transform between $i$ and $j$ (computed once from scan matching or odometry, *before* optimization), and $\boldsymbol{\Omega}_{ij}$ is its information matrix (how much to trust this measurement)

* **Error:**

$$
\mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)
$$


  The error for this edge — the gap between what was measured ($\mathbf{z}_{ij}$) and what the current pose estimates $x_i$, $x_j$ imply the transform should be.

**Goal:** adjust all node poses simultaneously to minimize the total weighted error across every edge — each edge "pulls" its two poses toward agreeing with its measurement, in proportion to how trusted ($\boldsymbol{\Omega}_{ij}$) it is:

$$
\mathbf{x}^{*} = \arg\min_{\mathbf{x}} \sum_{ij} \mathbf{e}_{ij}^{T}\, \boldsymbol{\Omega}_{ij}\, \mathbf{e}_{ij}
$$

![Pose Graph Error](/assets/module-c/lecture-9/pose-graph-error.png)

This error function is exactly the form suited to **least squares error minimization**.

## Sparse Pose Adjustment

The task of pose-graph optimization is to seek a configuration of the nodes that **maximizes the likelihood of the measurements** encoded in the constraints. We use **Sparse Pose Adjustment (SPA)** to find the graph configuration.

This is the same least squares problem from the pose graph:

$$
\mathbf{x}^*=\arg\min_{\mathbf{x}}\sum_{ij}e_{ij}^{\top}\Omega_{ij}e_{ij}
$$

SPA provides an efficient way to solve this optimization for a large, sparse pose graph.

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

This compares the measured relative transformation with the relative transformation predicted by our current pose estimates.

* $\bar z_{ij}$: the measured offset between $c_i,c_j$, from scan matching or odometry
* $h(c_i,c_j)$: the relative transformation implied by the current poses
* $e_{ij}$: how much the current poses violate this constraint

### Total Error

$$
\chi^2(\mathbf{c}, \mathbf{p}) \equiv \sum_{ij} e_{ij}^{\top}\, \Lambda_{ij}\, e_{ij}
$$

- $\Lambda_{ij}$: precision matrix

The optimal placement of each $c$ is found by **minimizing the total error**. This is a least squares problem, solved using the **Levenberg-Marquardt (LM)** framework.

![Sparse Pose Adjustment Total Error](/assets/module-c/lecture-9/sparse-pose-adjustment-total-error.png)

## Scan Matching Methods

Graph-based SLAM is also **scan (sensor) matching method agnostic**:
- ICP
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
- **Formally:**

$$
\min \sum_{ij} \mathbf{e}_{ij}^T \boldsymbol{\Omega}_{ij} \mathbf{e}_{ij}
$$

an overdetermined least squares problem solved via Sparse Pose Adjustment with Levenberg-Marquardt.
- Graph-based SLAM is **sensor and scan-matcher agnostic**. It only needs one sensor for relative transformations and one for absolute environment observation
