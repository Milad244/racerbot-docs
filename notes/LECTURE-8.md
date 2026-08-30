# Lecture 8 Notes

**Lecture 8:** https://www.youtube.com/watch?v=SRBdpoPl57Q

## Localization and Mapping: Particle Filters

Lesson plan:
1. Map making with Hector SLAM
2. Particle Filter Localization
3. Adaptive Monte Carlo Localization (AMCL)
4. Recursive Bayes Filtering used in AMCL
5. Tuning Particle Filters in ROS

The lecture runs in two passes: **Intuition** first, then **Analysis**.

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

![Scan Matching Frame Alignment](/assets/module-c/lecture-8/scan-matching-frame-alignment.png)

### The Hector SLAM Objective
The robot pose is $\boldsymbol{\xi} = (p_x, p_y, \psi)^{\mathrm{T}}$. Hector SLAM solves:

$$
\boldsymbol{\xi}^{*} = \arg\min_{\boldsymbol{\xi}} \sum_{i=1}^{n} \left[ 1 - M(\mathbf{S}_i(\boldsymbol{\xi})) \right]^2
$$

- $\mathbf{S}_i(\boldsymbol{\xi})$: impact coordinates of the $i$-th scan in the world frame
- $M(\mathbf{S}_i(\boldsymbol{\xi}))$: map value $\{0, 1\}$ at the coordinates given by $\mathbf{S}_i$
- $n$: total number of scans

A perfect alignment puts every scan endpoint on a wall ($M = 1$), driving the sum to zero. With real uncertainty and minor errors, the goal is to **minimize** this summation.

![Hector Scan Matching Objective](/assets/module-c/lecture-8/hector-scan-matching-objective.png)

### Optimizing Over the Pose Change
The pose can be written as the previous pose plus the change in pose over the small time interval, which turns the minimization into a function over $\Delta\boldsymbol{\xi}$:

$$
\sum_{i=1}^{n} \left[ 1 - M(\mathbf{S}_i(\boldsymbol{\xi} + \Delta\boldsymbol{\xi})) \right]^2 \rightarrow 0
$$

Expand using the **Taylor expansion** of the function $M$:

$$
\sum_{i=1}^{n} \left[ 1 - M(\mathbf{S}_i(\boldsymbol{\xi})) - \nabla M(\mathbf{S}_i(\boldsymbol{\xi})) \frac{\partial \mathbf{S}_i(\boldsymbol{\xi})}{\partial \boldsymbol{\xi}} \Delta\boldsymbol{\xi} \right]^2 \rightarrow 0
$$

Solving for $\Delta\boldsymbol{\xi}$ yields the **Gauss-Newton equation**. Evaluating it gives a step $\Delta\boldsymbol{\xi}$ that minimizes the objective function.

In practice: keep rotating and translating the second scan until the error is minimum, which means the scans are aligned. Then update the pose with the calculated pose change, transform the current laser scans to the new pose, and update the map with the transformed scans.

![Hector SLAM Gauss-Newton Derivation](/assets/module-c/lecture-8/hector-gauss-newton.png)

### Map Update in Practice
The occupancy grid does **not** assign a wall or free space from a single scan, since that would introduce a lot of noise. Each cell maintains a value that is a function of:
- how many times the cell was encountered by a scan as free or occupied, and
- the probability of the scan measurement being correct

After every few laser scans, once the map is confident about a chunk of cells, it gets published as the occupancy map.

### Multi-Resolution Map Representation
This kind of optimization can get stuck in **local minima**. To avoid this, rather than using a single occupancy grid, the equation is optimized first over coarser maps, and that estimate is fed as the input to the optimization over the higher-resolution map.

Example: with a target resolution of 5 cm and 3 multi-resolution grids, the algorithm iterates over grids of **20 cm → 10 cm → 5 cm**. The pose update from the 20 cm grid is the input to the 10 cm grid, and so on.

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
- `update_factor_free`: log odds probability for free cells
- `update_factor_occupied`: log odds probability for occupied cells

## Localization with Odometry

### First Attempt: Motion Integration
**Odometry:** start at a known pose and integrate control and motion measurements to estimate the current pose.
- Integrate dynamics using information from the VESC, wheel encoders, IMU, etc.
- Odometry = **open loop estimation** = error increases over time

By measuring the number of rotations on the left and right wheels we can calculate the distance travelled from an initial position and predict the car's pose with *some* certainty. But as it integrates distance over time, error keeps accumulating and the odometry drifts.

Walking through it: start at a known pose → uncertainty grows with every step → eventually you are **no longer able to determine the correct position**.

![Odometry Drift](/assets/module-c/lecture-8/odometry-drift.png)

### Wheel Spin Due to Lack of Traction
Wheel-encoder-based odometry does not account for **wheel slippage**, which is very common on high-speed platforms. Due to the high torque of the motors, the wheels spin in place before the car actually starts to move. The odometry believes the car has moved even though the wheel is spinning in the same location, yielding an incorrect pose.

![Wheel Slip](/assets/module-c/lecture-8/wheel-slip.png)

### Mapping with Odometry Meets Reality
- Motion is noisy, we cannot ignore it
- Assuming known poses fails
- Often, the sensor itself is rather precise

An open-loop run with no feedback (IMU + wheel encoders only) produces a badly distorted building layout.

![Mapping with Odometry Meets Reality](/assets/module-c/lecture-8/mapping-with-odometry-reality.png)

### Scan Matching Also Fails Alone
Scan-matching-based odometry can fail to record movement down a long corridor because consecutive scans are nearly identical. It only realizes the movement once a prominently different scan appears at a turn.

### The Issue with Localization
We need:
1. A mechanism to compensate accumulated odometry error from wheel slippage and scan matching failure (among other uncertainties)
2. A solution robust enough to compensate for a lack of information on the initial position, handling inaccuracies in the initial pose

**Solution: Monte Carlo Localization** (uses histograms / sample sets)
**Alternate solutions:** Kalman Filter (uses Gaussians to track position), Topological Markov Localization

### Pose Correction using Scan Matching
Maximize the observation likelihood of the current pose relative to the previous pose and the map, combining the **sensor observation model** with the **motion odometry model**.

## Particle Filter: Intuition (1D Example)

Setup: a robot moving in one dimension along a corridor with a few doors. It senses only doors, and has odometry telling it how far it moved.

### Belief State
Initially the robot has no information about where it is. The graph of position (x-axis) versus probability of being there is the **belief state**: the probability of the robot being at that particular position.

### $t = 1$: Sense a Door
The robot senses a door. The probability of receiving this sensor measurement along the corridor is $p(z \mid x)$, where $z$ is the sensor measurement (door or not) and $x$ is the position. This is the **measurement model**.

### Belief State Update
Combining the measurement model with the prior gives the **posterior belief**:
- The robot has an increased belief of being next to a door (bumps at each door)
- There is still a residual probability of being in the places between the bumps

![Belief State Measurement Update](/assets/module-c/lecture-8/belief-state-measurement-update.png)

### $t = 2$: Move Forward
Update the belief state by shifting it forward the same distance, with some added noise to account for odometry error. This is the **motion update** of the belief state.

Since robot motion is uncertain, the bumps come out **flatter** than before.

### Measurement Update Again
Sense a door again (now the 2nd door). Convolving this measurement with the previous belief state gives a new belief state with a high probability of being near door 2.

**At this point the robot has localized itself.**

![Belief State After Localizing](/assets/module-c/lecture-8/belief-state-localized.png)

### From Continuous to Discrete
The belief state is a distribution that focuses most of its weight onto the correct hypothesis. But using it as a continuous function is computationally expensive.

Instead it is **sampled** into discrete points. Each sample position is called a **particle**. The entire belief state becomes a discrete set of particle positions, each associated with the probability of the robot being at that position.

Where the probability is higher (near the middle door), the particles are drawn more densely. Each drawn particle carries a **weight** equal to the probability of the belief state at that location.

![Belief State as Discrete Particles](/assets/module-c/lecture-8/belief-state-discrete-particles.png)

## Particle Filter in 2D

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

![Scan Correlation Formula](/assets/module-c/lecture-8/scan-correlation-formula.png)

Repeat for each particle and record the correlation score. Particles that align well with the map get a high correlation score. The particle with the **maximum weight** is chosen as the pose of the robot at the current state.

![Scan Correlation Weights](/assets/module-c/lecture-8/scan-correlation-weights.png)

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

Because weights are multiplied over iterations, particles with small weights shrink to a very small value over time, leaving the filter with effectively very few useful particles. Only a few particles become prominent.

To avoid this, **resample** the particles when the effective particle count becomes too few, and reset all the weights.

How it works:
- Start with an initial set of particles drawn from the distribution
- As they propagate, their weights change and a few become dominant (larger size = larger weight)
- When those weights get very large, the particles must be resampled
- Resampling draws **more particles close to the particles with higher weight**, so the new particle set has higher density near where the high-weight particles were
- All new particles are drawn with the **same weight**, so the particle filter begins from scratch

![Resampling](/assets/module-c/lecture-8/resampling.png)

## KLD Sampling and AMCL

This form of localization is computationally expensive because the correlation must be calculated iteratively for each particle. The more particles, the longer it takes to localize.

**Kullback-Leibler divergence (KLD) sampling** optimizes this by controlling the number of particles based on the difference between odometry and particle-based localization:
- Initially, when the position is unknown, the particle cloud is large due to position uncertainty
- As the car moves, the particles converge and the cloud shrinks
- KLD sampling culls redundant particles and improves performance

![KLD Sampling](/assets/module-c/lecture-8/kld-sampling.png)

Properties:
- Variable particle size
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
4. AMCL pose: the corrected pose of our vehicle
5. Particle cloud, for debugging

## Particle Filters: Analysis

### Problem Definition
- Estimating the state of a dynamical system is a fundamental problem. Here, that means estimating the state of the vehicle within its environment
- The **recursive Bayes filter** is an effective approach to estimate the belief about the state of a dynamical system
  - How do we represent this belief in a probabilistic way?
  - How do we maximize it?
- Particle filters efficiently represent an **arbitrary (non-Gaussian)** distribution
- Basic principle: use sampling to capture a discrete approximation of the belief set through a set of hypotheses (particles)
  - A set of state hypotheses ("particles")
  - **Survival of the fittest** to concentrate our finite particles so they better sample the arbitrary distribution of the belief

### Starting with the Bayes Filter
The key idea of Bayes filtering is to estimate a probability density over the state space conditioned on the data. This posterior is called the **belief**:

$$
Bel(x_t) = p(x_t \mid d_{0 \dots t})
$$

- $x_t$: robot state at time $t$
- $d_{0 \dots t}$: the data from time 0 to $t$

Bayes filters assume the environment is **Markov**: past and future data are conditionally independent if one knows the current state.

For mobile robots we distinguish two types of data: **perceptual** data such as laser range measurements ($o$ for observation), and **odometry** data carrying information about robot motion ($a$ for action):

$$
Bel(x_t) = p(x_t \mid o_t, a_{t-1}, \dots, o_0)
$$

### Observation Update
Use Bayes rule, $p(A \mid B) = \dfrac{p(B \mid A)p(A)}{p(B)}$, to rewrite the belief:

$$
Bel(x_t) = \frac{p(o_t \mid x_t, a_{t-1}, \dots, o_0)\,p(x_t \mid a_{t-1}, \dots, o_0)}{p(o_t \mid a_{t-1}, \dots, o_0)}
$$

$$
Bel(x_t) = \eta\, p(o_t \mid x_t, a_{t-1}, \dots, o_0)\, p(x_t \mid a_{t-1}, \dots, o_0)
$$

![Observation Update](/assets/module-c/lecture-8/observation-update.png)

### Markov Assumption
Bayes filters rest on the assumption that future data is independent of past data given knowledge of the current state. Using it, the belief based on the last perceptual measurement simplifies to:

$$
Bel(x_t) = \eta\, p(o_t \mid x_t)\, p(x_t \mid a_{t-1}, \dots, o_0)
$$

### Odometry Update
What if we just received an odometry measurement instead?

$$
Bel(x_t) = p(x_t \mid x_{t-1}, a_{t-1}, \dots, o_0)
$$

Rewrite via the **Law of Total Probability**:

$$
Bel(x_t) = \int p(x_t \mid x_{t-1}, a_{t-1}, \dots, o_0)\, p(x_{t-1} \mid a_{t-1}, \dots, o_0)\, dx_{t-1}
$$

Apply the Markov assumption:

$$
Bel(x_t) = \int p(x_t \mid x_{t-1}, a_{t-1})\, Bel(x_{t-1})\, dx_{t-1}
$$

![Odometry Update](/assets/module-c/lecture-8/odometry-update.png)

### Bayes Recursive Filter
After simplification, both steps combine into:

$$
Bel(x_t) = \eta\, p(o_t \mid x_t) \int p(x_t \mid x_{t-1}, a_{t-1})\, Bel(x_{t-1})\, dx_{t-1}
$$

Read it **right to left**:
- **Previous Belief** $Bel(x_{t-1})$: draw your particles with replacement according to importance weight
- **Transition / Motion Model** $p(x_t \mid x_{t-1}, a_{t-1})$: simulate noisy dynamics of particles based on control input
- The **integral**: integrate out over previous beliefs (in practice a finite approximation)
- **Observation Likelihood / Sensor Model** $p(o_t \mid x_t)$: compute how likely your measurements were given the updated particles
- **Normalization Constant** $\eta$: make sure everything adds up to 1

In practice we represent the distributions **non-parametrically** using particles (a finite set of samples).

![Bayes Recursive Filter](/assets/module-c/lecture-8/bayes-recursive-filter.png)

## Why Samples

### Gaussian Filters Are Limited
The Kalman filter and its variants can only model **Gaussian** distributions. The goal is an approach for dealing with **arbitrary** distributions.

### Key Idea: Weighted Samples
Use multiple **weighted** samples to represent arbitrary distributions.

**Particle set:** a set of weighted samples that represents the posterior, where each particle carries a state hypothesis and an importance weight.

**Particles for approximation:** the more particles fall into a region, the higher the probability of that region.

![Weighted Samples](/assets/module-c/lecture-8/weighted-samples.png)

### Importance Sampling Principle
Closed-form sampling is only possible for a few distributions (e.g. Gaussian). To sample from others:

- We can use a different distribution $\pi$ to generate samples from $f$
- Account for the "differences between $\pi$ and $f$" using a weight:

$$
\omega = \frac{f(x)}{\pi(x)}
$$

- **Target distribution:** $f$
- **Proposal distribution:** $\pi$
- **Pre-condition:** $f(x) > 0 \rightarrow \pi(x) > 0$

![Importance Sampling Principle](/assets/module-c/lecture-8/importance-sampling.png)

### Particle Filter for Dynamic State Estimation
- Recursive Bayes filter
- Non-parametric approach
- Models the distribution by samples
- **Prediction:** draw from the proposal
- **Correction:** weighting by the ratio of target and proposal

The more samples we use, the better the estimate.

## Particle Filter Algorithm

1. **Sample** the particles using the proposal distribution:

$$
x_t^{[j]} \sim proposal(x_t \mid \dots)
$$

2. **Compute the importance weights:**

$$
w_t^{[j]} = \frac{target(x_t^{[j]})}{proposal(x_t^{[j]})}
$$

3. **Resampling:** draw sample $i$ with probability $w_t^{[i]}$ and repeat $J$ times

![Particle Filter Algorithm](/assets/module-c/lecture-8/particle-filter-algorithm.png)

The three steps map onto: **Prediction Step → Correction Step → Re-sampling Step**, taking as input the old sample set, controls, and observations.

### Monte Carlo Localization
- Each particle is a **pose hypothesis**
- The **proposal** is the motion model
- **Correction** is via the observation model

### Resampling Restated
- Draw sample $i$ with probability $w^{[i]}$, repeat $J$ times
- Informally: "replace unlikely samples by more likely ones"
- **Survival of the fittest**
- A "trick" to avoid many samples covering unlikely states
- Needed because we have a limited number of samples

### Global Localization Walkthrough
Starting from initialization with **10,000 particles** spread over the whole map, the loop is:

**first observation → weight update → resampling → motion update → measurement → weight update → resampling → motion update → ...**

Over these cycles the particle cloud collapses from covering the entire map down to the true pose.

![Particle Filter for Localization in a Known Map](/assets/module-c/lecture-8/particle-filter-known-map.png)

## Tuning Particle Filters

Find these in `localize.launch`:
- **Number of Particles:** 250-10000
- **Angle Step:** downsamples the range measurements
- **Squash factor:** smooths particle weights
- **Max range:** 10-30 meters, good for getting rid of useless measurements with no returns, floor, etc.
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