# Lecture 8 Notes

**Lecture 8:** https://www.youtube.com/watch?v=SRBdpoPl57Q

## Localization and Mapping: Particle Filters

Lesson plan:
1. Map making with Hector SLAM
2. Particle Filter Localization
3. Adaptive Monte Carlo Localization (AMCL)
4. Tuning Particle Filters in ROS

## Why We Need a Map

We need a map to know **where the car is and what is around it** for localization and planning.

* Basic path planning gives **turns, not trajectories**, which is not enough for racing
* Racing requires a precise **race line** and high speed
* The optimal line for one turn depends on the **next turn**: a line that is best for the current turn may make the next turn worse
* In F1 races the track is known in advance → we should **build the map in advance too**

### System Overview
The full stack, in order:

**Mapping → Localization → Path Planning → Control**

- Mapping is handled by **Hector SLAM**
- Localization is handled by **Adaptive Monte Carlo Localization (AMCL)**

## SLAM: A Chicken-and-Egg Problem

You need a map of the environment to determine the robot's location, **but** you need the robot's location first to build the map itself.

![SLAM Chicken-Egg Problem](/assets/module-c/lecture-8/slam-chicken-egg.png)

SLAM (Simultaneous Localization and Mapping) resolves this by doing both at once, incrementally:

1. **1st scan:** register the initial map from the first scan and an initial pose estimate
2. **Motion update:** the car moves by a minor amount in a small time interval → new laser scan + an estimate of the car's motion
3. **Pose change:** correlate the current laser scan with the map observed so far to estimate the change in position
4. **Map update:** use the new pose estimate to update the map

This process repeats over many time samples, updating the map at each step.

![Overview of SLAM](/assets/module-c/lecture-8/slam-overview.png)

## Occupancy Grid Mapping

### Discrete Cells, Discrete States
Discretize the space into grid cells. Each cell value can be:
- **Occupied:** confirmed LiDAR hits over several iterations
- **Free:** LiDAR scanned and passed through
- **Unexplored:** LiDAR scans never reached it

![Occupancy Grid Cells](/assets/module-c/lecture-8/occupancy-grid-cells.png)

### Measurements
We maintain a separate database of laser measurements corresponding to each cell:

$$
m_{x,y} = \begin{cases} 1, & \text{LiDAR ray end point} \\ 0, & \text{free space between car and end point} \end{cases}
$$

### Map Cell States
The map state $Z$ is distinct from the raw LiDAR measurement $m_{x,y}$:

| $Z$ | Map cell state |
|----|----------------|
| $1$ | Occupied |
| $0$ | Unexplored |
| $-1$ | Free |

### Measurement Model
The underlying cell state is binary:

$$
z \in \lbrace -1,1 \rbrace
$$

where $z=-1$ means free and $z=1$ means occupied. Unexplored is not a third physical state; it means we have no information about the cell yet. Therefore, an unexplored cell starts with an initial belief of approximately:

$$
p(z=1)=p(z=-1)=\frac{1}{2}
$$

LiDAR measurements then provide evidence that updates this belief toward $z=-1$ or $z=1$.

### Log Odds Probability
For each LiDAR measurement, we use Bayes' rule to update our belief about the cell state:

$$
p(z \mid m_{x,y}) = \frac{p(m_{x,y} \mid z)p(z)}{p(m_{x,y})}
$$

Instead of repeatedly working directly with probabilities, we can express our belief using **odds**. Odds compare how likely the cell is to be in the state $z$ versus not being in that state:

$$
\text{odds}(z)=\frac{p(z)}{1-p(z)}
$$

For example, if $p(z)=0.8$, then the odds are $0.8/0.2=4$, meaning $z$ is 4 times more likely than $\neg z$.

Using odds, Bayesian updating becomes multiplication of the previous odds by the measurement's **likelihood ratio**. This ratio tells us how much the measurement $m_{x,y}$ supports $z$ over $\neg z$:

$$
\text{odds}(z \mid m_{x,y}) = \text{odds}(z) \frac{p(m_{x,y} \mid z)}{p(m_{x,y} \mid \neg z)}
$$

Taking the logarithm converts multiplication into addition:

$$
\log\text{odds}(z \mid m_{x,y}) = \log\text{odds}(z) + \log\frac{p(m_{x,y} \mid z)}{p(m_{x,y} \mid \neg z)}
$$

We accumulate evidence from multiple LiDAR measurements using addition instead of repeatedly multiplying small probabilities. Since the measurement likelihood $p(m_{x,y}\mid z)$ is less than 1, repeatedly multiplying it makes the result shrink toward zero, eventually causing numerical underflow.

To avoid this we work in **log odds**:

$$
\log odd_{\text{occ}} := \log \frac{p(z = 1 \mid m_{x,y} = 1)}{p(z = 1 \mid m_{x,y} = 0)}
$$

$$
\log odd_{\text{free}} := \log \frac{p(z = -1 \mid m_{x,y} = 0)}{p(z = -1 \mid m_{x,y} = 1)}
$$

At each time stamp we update the robot pose and map these constants to particular map cells, providing evidence about whether each cell is occupied or free.

### Map Update
Accumulate the log-odds evidence for each cell over iterations:

- If $m_{x,y}=1$ (LiDAR ray ends in the cell):

$$
\log \text{odd} = \log \text{odd} + \log \text{odd}_{\text{occ}}
$$

- If $m_{x,y}=0$ (LiDAR ray passes through the cell):

$$
\log \text{odd} = \log \text{odd} - \log \text{odd}_{\text{free}}
$$

The accumulated log odds represent our current belief about the cell state $z$.

### Accumulating Confidence and Saturation
In robotics, sensor measurements and robot motion are uncertain, so we should never become completely certain that an observation is correct. As measurements accumulate, however, the log odds can become extremely large or small, making the cell difficult to change with later evidence.

To prevent this, we apply upper and lower saturation limits:

$$
L_{\min}\leq \log odds \leq L_{\max}
$$

This limits our confidence so future measurements can still change the cell's state.

![Log Odds Saturation Limit](/assets/module-c/lecture-8/log-odds-saturation.png)

### Determining the Map State

After accumulating the log-odds, we can convert them back to $P(z=1)$ and use thresholds to classify the cell. For example:

$$
P(z=1)>0.5 \Rightarrow \text{Occupied}
$$

$$
P(z=1)<0.5 \Rightarrow \text{Free}
$$

$$
P(z=1)=0.5 \Rightarrow \text{Unknown}
$$

## Scan Matching with Hector SLAM

### Registering the First Scan
Take the initial pose of the robot as $(0, 0)$ and update the map cells using the first laser scan. The initial map is updated directly using the log probabilities for occupied and free cells.

Once the car proceeds further, the goal is to find the **pose change** relative to the previous measurement, by finding the transformation between the new laser scan and the previously registered map.

### Frame Alignment
- At $t = t_1$ we have a scan taken with respect to the car's pose at $t_1$
- The robot moves a small amount and turns; at $t = t_2$ we get a new scan
- Scan matching finds the pose change between these two timestamps by aligning the second scan with the first
- The change in robot pose is maintained *while* the map is updated simultaneously

**Initial pose** → **First LiDAR scan** → **Initial map**

**Robot moves** → **New LiDAR scan** → **Align scan with existing map** → **Estimate pose change** → **Update pose & map**

![Scan Matching Frame Alignment](/assets/module-c/lecture-8/scan-matching-frame-alignment.png)

### The Hector SLAM Objective
The robot pose is $\boldsymbol{\xi} = (p_x, p_y, \psi)^{\mathrm{T}}$, where $p_x,p_y$ are the robot's position and $\psi$ is its orientation. Hector SLAM finds the pose that makes the **new LiDAR scan align best with the existing map**:

$$
\boldsymbol{\xi}^{*} = \arg\min_{\boldsymbol{\xi}} \sum_{i=1}^{n} \left[ 1 - M(\mathbf{S}_i(\boldsymbol{\xi})) \right]^2
$$

- $\boldsymbol{\xi}$: candidate robot pose $(p_x,p_y,\psi)$ being tested
- $\boldsymbol{\xi}^{*}$: pose that gives the best alignment
- $\mathbf{S}_i(\boldsymbol{\xi})$: map-frame coordinates of the $i$-th LiDAR scan endpoint, given pose $\boldsymbol{\xi}$
- $M(\mathbf{S}_i(\boldsymbol{\xi}))$: map value at that endpoint's location; $M=1$ means occupied/wall and $M=0$ means free
- Any negative map values (e.g. free cells) are **masked to $0$** for this objective, so only occupied cells contribute as $M=1$.
- $n$: total number of LiDAR scan endpoints

The term

$$
\left[1-M(\mathbf{S}_i(\boldsymbol{\xi}))\right]^2
$$

is the **alignment error** for endpoint $i$. If the endpoint lands on a wall, $M=1$ and the error is $0$. If it lands in free space, $M=0$ and the error is $1$.

Therefore, Hector SLAM searches for the pose $\boldsymbol{\xi}$ that **minimizes the total error**, meaning the LiDAR endpoints line up as closely as possible with the occupied areas of the existing map.

![Hector Scan Matching Objective](/assets/module-c/lecture-8/hector-scan-matching-objective.png)

### Optimizing Over the Pose Change
Instead of testing many possible poses, Hector SLAM assumes the pose change is small and **linearizes the alignment function around the current pose**. The linearization tells us how small changes in pose affect the alignment error, and **Gauss-Newton uses this information to calculate the pose change $\Delta\boldsymbol{\xi}$**.


After the robot moves, we have a previous pose $\boldsymbol{\xi}$ and want to find the small **pose change** $\Delta\boldsymbol{\xi}$. The new pose is therefore:

$$
\boldsymbol{\xi}_{new}=\boldsymbol{\xi}+\Delta\boldsymbol{\xi}
$$

We want to find the $\Delta\boldsymbol{\xi}$ that makes the new LiDAR scan align best with the existing map:

$$
\sum_{i=1}^{n}
\left[
1-M(\mathbf{S}_i(\boldsymbol{\xi}+\Delta\boldsymbol{\xi}))
\right]^2
\rightarrow 0
$$

The problem is that $M(\mathbf{S}_i(\boldsymbol{\xi}+\Delta\boldsymbol{\xi}))$ is a nonlinear function of the pose. For a **small** pose change, we use a **first-order Taylor expansion (linearization)** to approximate how the map value changes when we slightly change the pose:

$$
\sum_{i=1}^{n}
\left[
1-M(\mathbf{S}_i(\boldsymbol{\xi}))
-\nabla M(\mathbf{S}_i(\boldsymbol{\xi}))
\frac{\partial\mathbf{S}_i(\boldsymbol{\xi})}{\partial\boldsymbol{\xi}}
\Delta\boldsymbol{\xi}
\right]^2
\rightarrow 0
$$

This turns the nonlinear alignment problem into a local linear approximation. **Gauss-Newton solves this approximation to calculate a $\Delta\boldsymbol{\xi}$ that reduces the alignment error.**

We update the pose:

$$
\boldsymbol{\xi}_{new}=\boldsymbol{\xi_{old}}+\Delta\boldsymbol{\xi}
$$

This process is repeated: after updating the pose, we linearize again around the new pose and calculate a new $\Delta\boldsymbol{\xi}$. This continues until the alignment error is sufficiently small.

![Hector SLAM Gauss-Newton Derivation](/assets/module-c/lecture-8/hector-gauss-newton.png)

### Map Update in Practice
The occupancy grid does **not** assign a wall or free space from a single scan, since that would introduce a lot of noise. Each cell maintains a value that is a function of:
- how many times the cell was encountered by a scan as free or occupied, and
- the probability of the scan measurement being correct

After every few laser scans, once the map is confident about a chunk of cells, it gets published as the occupancy map.

> **Note:** The map is used for scan matching even when it is still uncertain. As more LiDAR measurements accumulate, the probabilities provide stronger evidence about which cells are actually occupied or free.

### Multi-Resolution Maps

The optimization can get stuck in a **local minimum**: a pose where the alignment error is low, but it is **not the best possible alignment**.

To reduce this risk, Hector SLAM uses multiple versions of our **occupancy map**, each with a different resolution. A **coarser map** uses larger grid cells, which smooths out small details and makes it easier to find the general alignment.

The optimization proceeds from coarse to fine:

$$
20\text{ cm} \rightarrow 10\text{ cm} \rightarrow 5\text{ cm}
$$

The pose found using the 20 cm map is used as the starting point for the 10 cm map, which is then refined using the 5 cm map.

![Multi-Resolution Map Representation](/assets/module-c/lecture-8/multi-resolution-map.png)

## Saving the Map

ROS package called **Map Server**, which allows saving a map currently being published over the `/map` topic.

```bash
# Save the map
rosrun map_server map_saver [-f mapname]

# Load the map
rosrun map_server map_server <name.yaml>
```

## Odometry using Hector Mapping

During Hector SLAM we get a change in pose at each timestamp from the laser scan analysis, so it doubles as a **source of odometry**.

- Use Hector SLAM for measuring $\Delta\boldsymbol{\xi}$ while discarding the map
- Optional approach: **CSM (Canonical Scan Matcher)** by Andrea Censi, which does scan matching between 2 scans

### TF Tree
**Map Frame → Odom Frame → Base Frame → Laser Frame**

Hector odometry provides the transform between the odom frame and the base frame.

![Hector TF Tree](/assets/module-c/lecture-8/hector-tf-tree.png)

### Parameters for Hector SLAM in ROS
- `map resolution`: grid resolution
- `map_update_distance_thresh`: minimum distance travelled before a map update
- `map_update_angle_thresh`: minimum angle travelled before a map update
- `laser_max_dist`: laser sensor specification
- `update_factor_free`: factor controlling the **log-odds update strength** when updating free cells
- `update_factor_occupied`: factor controlling the **log-odds update strength** when updating occupied cells

## Localization with Odometry

### Motion Integration

Odometry: estimates pose by integrating motion measurements from wheel encoders, IMU, VESC, etc. Small errors accumulate over time, causing odometry drift.

Flow: known pose → integrate motion → errors accumulate → inaccurate pose.

### Wheel Spin Due to Lack of Traction
Wheel-encoder-based odometry does not account for **wheel slippage**, which is very common on high-speed platforms. Due to the high torque of the motors, the wheels spin in place before the car actually starts to move. The odometry believes the car has moved even though the wheel is spinning in the same location, yielding an incorrect pose.

### Scan Matching Also Fails Alone
Scan-matching-based odometry can fail to record movement down a long corridor because consecutive scans are nearly identical. It only realizes the movement once a prominently different scan appears at a turn.

### The Issue with Localization
We need:
1. A mechanism to compensate accumulated odometry error from wheel slippage and scan matching failure (among other uncertainties)
2. A solution robust enough to compensate for a lack of information on the initial position, handling inaccuracies in the initial pose

**Solution: Monte Carlo Localization** (uses histograms / sample sets)

**Alternate solutions:** Kalman Filter (uses Gaussians to track position), Topological Markov Localization

### From Continuous to Discrete
A continuous belief distribution is computationally expensive, so it is approximated using discrete particles.

Each particle represents a possible robot position.
Particles are denser where probability is higher.
Each particle has a weight representing the probability of that position.

> More details on Particle Filters can be found in [Lecture 7](/notes/LECTURE-7.md#particle-filter).

## Particle Filter

Using a map previously generated with Hector mapping, where black pixels are walls and grey pixels are free space:

1. The **red arrow** is the pose from odometry, which is not accurate
2. Generate a set of hypotheses: discrete particles drawn from a Gaussian with **mean = the odometry pose** and some **covariance** defining the size of the region to draw from. This yields a set of poses obtained by adding noise to the odometry pose
3. For each particle, orient the laser scan with respect to that particle's pose and compute a **correlation score**

![Particle Filter 2D Hypotheses](/assets/module-c/lecture-8/particle-filter-2d-hypotheses.png)

### Scan Correlation
The correlation score $S$ is:

$$
S = \frac{\sum_m \sum_n (A_{mn} - \bar{A})(B_{mn} - \bar{B})}{\sqrt{\left(\sum_m \sum_n (A_{mn} - \bar{A})^2\right)\left(\sum_m \sum_n (B_{mn} - \bar{B})^2\right)}}
$$

- $A$: the map, $B$: the scan
- $\bar{A}$, $\bar{B}$: mean values of all the pixel values of the respective maps, where walls have a value of 1 and free space has a value of 0
- $m$, $n$: the x and y coordinates of the pixels

This equation **counts the number of wall pixels that overlap** between the scan and the map.

Repeat for each particle and record the correlation score. Particles that align well with the map get a high correlation score. The particle with the **maximum weight** is chosen as the pose of the robot at the current state.

The drift in the odometry pose is visibly corrected by the particle filter.

![Odometry vs. Particle Filter](/assets/module-c/lecture-8/odometry-vs-particle-filter.png)

### Update Step
Repeat this correlation procedure for each sensor update:
- Update the particle cloud with the update in position from the odometry (propagate each particle by the distance and direction from odometry)
- Repeat the scan matching process for each particle and determine the weights

### Particle Weights
Weights are tracked by **normalizing and multiplying over time steps**. The score at time $t$ is first normalized by dividing by the largest score at time $t$, then multiplied with the previous weight for that particle at $t-1$:

$$
W_t \leftarrow W_{t-1} \times S
$$

This tracks how well each particle performs over time, so more particles can be drawn from that neighbourhood when needed.

![Particle Weights Update](/assets/module-c/lecture-8/particle-weights-update.png)

## Resampling

Repeated weight updates can cause particle depletion, where only a few particles retain significant weight.

Resampling: redraw particles proportional to their weights, concentrating them around high-probability regions and resetting their weights equally.

## KLD Sampling and AMCL

This form of localization is computationally expensive because the correlation must be calculated iteratively for each particle. The more particles, the longer it takes to localize.

**Kullback-Leibler divergence (KLD) sampling** optimizes this by controlling the number of particles based on the difference between odometry and particle-based localization:
- Initially, when the position is unknown, the particle cloud is large due to position uncertainty
- As the car moves, the particles converge and the cloud shrinks
- KLD sampling culls redundant particles and improves performance

Properties:
- Variable particle count
- Sample size is proportional to the error between the odometry position and the sample-based approximation
- i.e. smaller sample size once particles have converged

This is **Adaptive Monte Carlo Localization (AMCL)**.

### Particle Filters in ROS
- Adaptive Monte Carlo Localization package
- Localization for a robot moving in 2D space
- Localizes against a **pre-existing** map

The AMCL package provides a transform to correct the drift in Hector odometry so the car localizes accurately on the map.

### Input and Output Parameters
**Inputs:**

1. Laser scan
2. Dead reckoning / odometry source (Hector odometry)
3. Pre-built map

**Outputs:**

1. AMCL pose: the corrected pose of our vehicle
2. Particle cloud, for debugging

## Why Samples

### Gaussian Filters Are Limited

Kalman filters are limited because they:

* Assume a **Gaussian** belief distribution.
* Cannot naturally represent **multiple distinct hypotheses**.
* Require a reasonably **known initial position**.
* Struggle with highly **nonlinear or non-Gaussian** situations.

### Key Idea: Weighted Samples
Use multiple **weighted** samples to represent arbitrary distributions.

**Particle set:** a set of weighted samples that represents the posterior, where each particle carries a state hypothesis and an importance weight.

**Particles for approximation:** the more particles fall into a region, the higher the probability of that region.

![Weighted Samples](/assets/module-c/lecture-8/weighted-samples.png)

### Global Localization Walkthrough
Starting from initialization with **10,000 particles** spread over the whole map, the loop is:

**first observation → weight update → resampling → motion update → measurement → weight update → resampling → motion update → ...**

Over these cycles the particle cloud collapses from covering the entire map down to the true pose.

![Particle Filter for Localization in a Known Map](/assets/module-c/lecture-8/particle-filter-known-map.png)

## Tuning Particle Filters

Find these in `localize.launch`:
- **Number of Particles:** 250–10000 particles used for localization.
- **Angle Step:** controls how many LiDAR measurements are skipped when processing a scan; larger step → fewer measurements.
- **Squash factor:** smooths particle weights to reduce extreme differences between particles.
- **Max range:** 10–30 m. Longer ranges can help early localization by capturing more unique scene features, while shorter ranges reduce noisy/useless measurements such as floor returns or missing returns.
- **Method:** use RM-GPU

### ROS: Map Server
Edit `map_server.launch`:

```xml
<launch>
   <arg name="map" default="$(find particle_filter)/maps/your_map.yaml"/>
   <node pkg="map_server" name="map_server" type="map_server" args="$(arg map)" />
</launch>
```

Then add `your_map.pgm` and `your_map.yaml` to:

```
~/path/to/your_workspace/src/particle_filter/maps/
```

### AMCL Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `min_particles` | 100 | Minimum number of particles used for calculating correlation |
| `max_particles` | 500 | Maximum number of particles used for calculating correlation |
| `update_min_d` | 0.2 m | Minimum translation movement required before a pose update is published |
| `initial_pose_x` / `initial_pose_y` / `initial_pose_a` | 0 | The initial mean position of the particles used to initialize the filter |
| `initial_cov_xx` / `initial_cov_yy` / `initial_cov_aa` | 0 | The covariance of particles distributed around the mean |

## Key Takeaways
- SLAM is a chicken-and-egg problem: you need a map to localize, but a pose to map. It is solved by doing both incrementally
- Occupancy grids store cells as occupied / free / unexplored, updated in **log odds** to avoid numerical underflow, and **saturated** so no cell becomes absolutely certain
- Hector SLAM finds the pose change by minimizing $\sum [1 - M(\mathbf{S}_i(\boldsymbol{\xi}))]^2$, solved via Taylor expansion → Gauss-Newton; multi-resolution grids (20 → 10 → 5 cm) avoid local minima
- Odometry alone is open-loop: wheel slip and integration error make the pose drift without bound; scan matching alone fails in featureless corridors
- Particle filters are non-parametric, recursive Bayes filters. The posterior is represented by a set of **weighted samples**, so arbitrary non-Gaussian distributions can be captured
- The loop is **predict** (motion model as proposal) → **correct** (weight by scan correlation against the map) → **resample** (survival of the fittest)
- Weights are normalized and multiplied over time ($W_t \leftarrow W_{t-1} \times S$); without resampling, weight collapses onto a few particles
- AMCL = particle filter + **KLD sampling**, which varies the particle count with the remaining uncertainty, so a converged cloud costs less compute
- The art is designing appropriate **motion and sensor models**; MCL is the gold standard for indoor mobile robot localization today
