# Lecture 7 Notes

**Lecture 7:** https://youtu.be/kQi5IGzvr0c?si=89jK2Q1ja-OfM-Ym & https://youtu.be/SRBdpoPl57Q?t=1266 from time 21:05

## Localization and Mapping: Introduction to Bayes Filter

Lesson plan:
1. Introduction to State Estimation
2. Recap of Probability and Bayes Rule
3. Recursive Bayes Filter
4. Variants of the Bayes Filter: KF/EKF/UKF and Particle Filter

## State Estimation

Recall the system from vehicle dynamics:

$$
\dot{x} = Ax + Bu, \quad y = Cx
$$

- **System Dynamic** (state + input → state evolution): how control inputs $u$ and the current state $x$ determine how the state evolves
- **Sensor Model** (system → output): how the state shows up as measurements $y$
  - Sensors: IMU (on VESC), LiDAR, GNSS (GPS), camera, ...

State Estimation runs this pipeline *backwards*: use the previous state, control inputs, and sensor outputs to estimate the internal state (e.g. the car's location).

![State Estimation Pipeline](/assets/module-c/lecture-7/state-estimation-pipeline.png)

### Dead Reckoning

**Definition:** Calculate the current position of the car from a previous position, plus estimates of speed, heading (direction), and elapsed time.

**Does it work?**
- If we had **perfectly accurate control inputs** and a **perfect kinematic model** → it would be fine
- But there is always **uncertainty** in the measurement and system model, which creates **cumulative errors**

Example: recall the VESC parameters tuned for the car. Can `ros2 topic echo /odom` give an accurate speed/position?

```
# erpm = speed_to_erpm_gain * speed + speed_to_erpm_offset
speed_to_erpm_gain: 4614.0
speed_to_erpm_offset: 0.0
# servo = steering_angle_to_servo_gain * steering_angle + steering_angle_to_servo_offset
steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.5304
```

These are imperfect estimates → we always have noise.

Key reframe: instead of asking for one exact position, frame state estimation as approximating a *distribution* over the position $p(x)$.

![Dead Reckoning Noise](/assets/module-c/lecture-7/dead-reckoning-noise.png)

### State Estimation as a Distribution

We approximate a distribution using two information sources:
- **Control Input** → **Prediction** (push the belief forward using motion)
- **Observation** → **Correction** (pull the belief toward what the sensors see)

## Recap of Probability and Bayes Rule

### Discrete and Continuous Probability
Probability describes how likely different outcomes are. Depending on whether the possible values are countable or continuous, we represent probability differently.

- **Discrete:** the variable can take a countable set of values (e.g. a dice roll). We use a **probability mass function (PMF)** to assign a probability to each possible value:

$$
P(X=x)
$$

  where $X$ is the random variable and $x$ is a particular value it can take. For example, $P(X=3)$ means "the probability that the dice roll is 3." The probabilities of all possible values add up to $1$.

- **Continuous:** the variable can take any value in a range (e.g. position). We use a **probability density function (PDF)** $f_X(x)$ to describe how probability is distributed.
  - The PDF itself is not a probability. Probability comes from the **area under the PDF** over an interval:

$$
P(a \leq X \leq b) = \int_a^b f_X(x)\,dx
$$

  - The **cumulative distribution function (CDF)** gives the probability that $X$ is less than or equal to a given value:

$$
F_X(x) = P(X \leq x) = \int_{-\infty}^{x} f_X(t)\,dt
$$

### Conditional Probability
- $P(B \mid A)$: the chance of event $B$ when event $A$ has already happened ("probability of $B$ given $A$")

### Bayes Rule
Starting from the two ways to write a joint probability:

$$
P(AB) = P(A)\,P(B \mid A) \quad (1)
$$
$$
P(AB) = P(B)\,P(A \mid B) \quad (2)
$$

Combining gives Bayes Rule:

$$
P(B \mid A) = \frac{P(AB)}{P(A)} = \frac{P(A \mid B)\,P(B)}{P(A)}
$$

In words:

$$
\text{posterior} = \frac{\text{likelihood} \times \text{prior}}{\text{evidence}}
$$

With $A$: evidence (observation), $B$: hypothesis (state):
- **Prior** $P(B)$: the probability distribution (belief) *before* the evidence is considered
- **Likelihood** $P(A \mid B)$: probability of the evidence given the belief → how compatible is the observation with this hypothesized state?
- **Posterior** $P(B \mid A)$: updated belief *after* the evidence is considered
- **Evidence** $P(A)$: usually a normalization term so the posterior is a valid PDF

### Law of Total Probability
Decompose a probability by conditioning on mutually exclusive and exhaustive cases.

- **Discrete case:** for mutually exclusive, exhaustive events $B_1, \dots, B_k$:

$$
P(A) = \sum_{i=1}^{k} P(A \cap B_i) = \sum_{i=1}^{k} P(A \mid B_i)\,P(B_i)
$$

- **Continuous case:**

$$
P(A) = \int_{-\infty}^{\infty} P(A \mid X = x)\,f_X(x)\,dx
$$

where $f_X(x)$ is the probability density function (PDF) and $F(x)$ the cumulative distribution function (CDF).

![Law of Total Probability](/assets/module-c/lecture-7/law-of-total-probability.png)

### More Evidence → More Conditions
Bayes Rule extends when we condition on extra context $C_1, \dots, C_n$:

$$
P(B \mid A, C_1, \dots, C_n) = \frac{P(A \mid B, C_1, \dots, C_n)\,P(B \mid C_1, \dots, C_n)}{P(A \mid C_1, \dots, C_n)}
$$

- Combining historical information → Recursive Bayes Filter
- Combining multiple sensor measurements → Sensor Fusion

The belief (posterior) over the robot state is updated recursively using the previous belief, control inputs, and observations:

$$
\text{Bel}(x_t) = P(x_t \mid o_t, u_t, o_{t-1}, u_{t-1}, \dots)
$$

- $x_t$: robot state
- $o_t$: current observation, $u_t$: control input
- $o_{t-1}, \dots$: history of observations and controls
- The previous state $x_{t-1}$ is incorporated through the recursive belief $Bel(x_{t-1})$

## Recursive Bayes Filter

### Hidden Markov Model (HMM)

**Goal:** Take the belief at time $t-1$ and advance the estimate of $x$ to time $t$.

Two models drive the chain:
- **Action / Motion model (transition):** $p(x_t \mid u_t, x_{t-1})$: likelihood of the next state, given current control and previous state
- **Sensor model:** $p(o_t \mid x_t)$: likelihood of the current observation, given the current state

### Markov Property (conditional independence)
- The current observation depends only on the current state:

$$
P(O_t \mid x_{1:t}, u_{1:t}) = P(O_t \mid x_t)
$$

- The current state depends only on the previous state and current control:

$$
P(x_t \mid x_{1:t-1}, u_{1:t}) = P(x_t \mid x_{t-1}, u_t)
$$

This is what collapses the full history into a simple recursion.

![Hidden Markov Model](/assets/module-c/lecture-7/hidden-markov-model.png)

### Step 1: Prediction (with the control input)
Like dead reckoning, but probabilistic. Start from the predicted belief $\overline{bel}(x_t)$ and apply the Law of Total Probability (condition on $x_{t-1}$):

$$
\overline{bel}(x_t) = P(x_t \mid o_{1:t-1}, u_{1:t})
$$

$$
= \int P(x_t \mid x_{t-1}, o_{1:t-1}, u_{1:t})\,\underbrace{P(x_{t-1} \mid o_{1:t-1}, u_{1:t})}_{\text{recursive term } = \, bel(x_{t-1})}\,d(x_{t-1})
$$

Apply the Markov property to simplify the first term to the action model:

$$
\overline{bel}(x_t) = \int \underbrace{P(x_t \mid x_{t-1}, u_t)}_{\text{action model}}\,\underbrace{bel(x_{t-1})}_{\text{posterior of } x_{t-1}}\,d(x_{t-1})
$$

### Step 2: Correction (with the observation)
Fold in the new observation $o_t$ using Bayes Rule:

$$
bel(x_t) = P(x_t \mid o_{1:t-1}, o_t, u_{1:t})
$$

$$
= \frac{P(o_t \mid o_{1:t-1}, x_t, u_{1:t})\,P(x_t \mid o_{1:t-1}, u_{1:t})}{P(o_t \mid o_{1:t-1}, u_{1:t})}
$$

<details>
<summary>Bayes Rule: What gets swapped?</summary>

Starting from Bayes Rule with additional context $C$:

Bayes Rule swaps $A$ and $B$ in the conditional while keeping $C$ fixed.

</details>


$$P(B \mid A,C)=\frac{P(A \mid B,C)\,P(B\mid C)}{P(A\mid C)}$$

Apply the Markov property to the likelihood ($o_t$ depends only on $x_t$):

$$
bel(x_t) = \frac{\overbrace{P(o_t \mid x_t)}^{\text{sensor model}}\;\overbrace{P(x_t \mid o_{1:t-1}, u_{1:t})}^{\text{prior } = \, \overline{bel}(x_t)}}{\underbrace{P(o_t \mid o_{1:t-1}, u_{1:t})}_{\text{normalization term}}}
$$

### Practical Issue
Both steps require multiplication and even integration of two probability distributions, generally intractable in closed form. Writing the normalizer as $\eta$:

$$
\overline{bel}(x_t) = \int P(x_t \mid x_{t-1}, u_t)\,bel(x_{t-1})\,d(x_{t-1})
$$
$$
bel(x_t) = \frac{P(o_t \mid x_t)\,\overline{bel}(x_t)}{P(o_t \mid o_{1:t-1}, u_{1:t})} = \eta\,P(o_t \mid x_t)\,\overline{bel}(x_t)
$$

This is why we use variants of the Bayes filter that make the math tractable.

![Bayes Filter Derivation](/assets/module-c/lecture-7/bayes-filter-derivation.png)

## Variants of Bayes Filter

The Bayes filter is general, but computing the belief exactly can be difficult. Different filters make different assumptions or approximations:

- **Gaussian-based:** KF, EKF, UKF — represent the belief with a Gaussian
- **Sampling-based:** PF — represent the belief with weighted samples

### Assume a Simple (Gaussian) Distribution
- **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, **Unscented Kalman Filter (UKF)**

Why Gaussians work so well:
- **Conditional distribution:** if two sets of variables are jointly Gaussian, the conditional of one given the other is again Gaussian
- **Self-conjugate:** a Gaussian likelihood with a Gaussian prior yields a Gaussian posterior

→ The distribution keeps the same form throughout propagation, giving an analytical solution.

### KF Example
- **State:** position of the car
- **Observation:** sensor measurement to a pole
- **Assumption:** both state and observation are Gaussian

![KF Example](/assets/module-c/lecture-7/kf-example.png)

Walkthrough:
- **[Fig1]** Initial knowledge at $T=0$ (known initial velocity)
- **[Fig2]** Prediction at $T=1$: the action model adds uncertainty → the position Gaussian gets wider (confidence decreases)
- **[Fig3]** A noisy measurement at $T=1$ (its own Gaussian)
- **[Fig4]** Multiply the prediction and measurement PDFs → fused estimate that is sharper than either alone

![KF Gaussian Fusion 1](/assets/module-c/lecture-7/kf-gaussian-fusion-1.png)

![KF Gaussian Fusion 2](/assets/module-c/lecture-7/kf-gaussian-fusion-2.png)

### Distinction Between Gaussian Filters

All three filters represent the belief as a Gaussian:

$$
x_t \sim \mathcal{N}(\hat{x}_t, P_t)
$$

- $x_t$: random variable representing the true state
- $\hat{x}_t$: estimated state (the mean of the Gaussian)
- $P_t$: covariance, representing uncertainty in the estimate

All three repeatedly **predict → correct**, but differ in how they handle the system dynamics and measurements.

#### KF
For a **linear** system:

$$
x_{t+1} = A x_t + B u_t + w_t
$$

The Gaussian remains Gaussian, so the mean and covariance can be propagated and updated directly:

$$
\hat{x}_{t+1}^- = A\hat{x}_t + Bu_t,
\qquad
P_{t+1}^- = AP_tA^\top + Q
$$

The correction uses the sensor model:

$$
o_{t+1} = Cx_{t+1} + v_{t+1}
$$

so the predicted sensor observation is:

$$
\hat{o}_{t+1} = C\hat{x}_{t+1}^-
$$

The KF compares the actual observation with the predicted observation:

$$
o_{t+1} - \hat{o}_{t+1}=o_{t+1} - C\hat{x}_{t+1}^{-}
$$

and uses this difference, together with the uncertainty $P_{t+1}^-$ and sensor noise covariance $R$, to update the mean and covariance:

$$
(\hat{x}_{t+1}^-,P_{t+1}^-)
\rightarrow
(\hat{x}_{t+1},P_{t+1})
$$

This is the KF's closed-form version of the Bayes correction: instead of checking individual possible states, it analytically updates the entire Gaussian.

No approximation is needed under the linear/Gaussian assumptions.

#### EKF
For a **nonlinear** system:

$$
x_{t+1} = f(x_t, u_t)
$$

The EKF locally approximates the nonlinear function as linear around the current estimate, using a Jacobian. It can then perform a KF-like update:

$$
\text{nonlinear function}
\rightarrow
\text{local linear approximation}
\rightarrow
(\hat{x}_{t+1}, P_{t+1})
$$

This approximation can become inaccurate when the system is strongly nonlinear.

#### UKF
The UKF also handles **nonlinear** systems:

$$
x_{t+1} = f(x_t, u_t)
$$

Instead of linearizing $f$, it selects a small set of representative **sigma points (carefully chosen points that represent the Gaussian)** from the Gaussian, propagates them through the nonlinear function, and uses the resulting points to estimate the new mean and covariance:

$$
(\hat{x}_t,P_t)
\rightarrow
\text{sigma points}
\rightarrow
f(\text{sigma points})
\rightarrow
(\hat{x}_{t+1},P_{t+1})
$$

So the main distinction is:

- **KF:** linear system → directly propagate the Gaussian
- **EKF:** nonlinear system → locally linearize it
- **UKF:** nonlinear system → propagate representative points through it

### EKF Example
- **GNSS** (Global Navigation Satellite System) → position measurements (e.g., GPS)

On a GNSS-tracked trajectory:
- Blue = true trajectory, Black = dead reckoning, Green = GNSS observations
- Red line = EKF estimate, Red ellipse = EKF covariance estimate

![EKF Trajectory](/assets/module-c/lecture-7/ekf-trajectory.png)

### Practical Problem of KF: Non-Gaussian Noise
When the noise is *not* Gaussian (e.g. GNSS in practice), the Gaussian filters (KF/EKF/UKF) can degrade, motivating the particle filter.

![KF Non-Gaussian Noise](/assets/module-c/lecture-7/kf-non-gaussian.png)

## Particle Filter

### Use a Sampling-Based Method

- **Particle Filter (PF)**: represents complicated, non-Gaussian distributions with samples

### Idea
Instead of assuming a specific distribution shape (e.g. a Gaussian), use a sample-based representation.

- **Monte Carlo method:** rely on repeated random sampling to obtain numerical results
- **Advantage:** can approximate complicated distributions
- More samples → the sampled histogram converges to the true PDF (10 → 50 → 100 → 1000 samples)

### Representing a Distribution by Sampling

**Unweighted samples (accept-reject sampling):**
- Uniformly draw many particles; accept those that fall under the PDF
- Each particle has equal weight; probability is encoded by the density of particles
- Issue: inefficient, many particles get thrown away

**Weighted samples (importance sampling):**
- We want to approximate a target distribution $p(x)$, but instead of sampling directly from it, we sample from an easier **proposal distribution** $q(x)$:

$$
x^{(i)} \sim q(x)
$$

- Each sampled $x^{(i)}$ is a possible value of the state. We then evaluate both distributions at that sampled value and compute its weight:

$$
w^{(i)} = \frac{p(x^{(i)})}{q(x^{(i)})}
$$

  - $x^{(i)}$: the $i$-th sampled state
- $q(x^{(i)})$: how likely the proposal was to generate that state; dividing by it corrects for the proposal's sampling bias
- $p(x)$: the target distribution we want to approximate.
- $w^{(i)}$: how important the sampled state should be, based on how likely it is under $p$ relative to $q$
- $p(x^{(i)})$: the value of the target distribution at the sampled state $x^{(i)}$ — i.e., how much probability density the target assigns to that state.
This is the theoretical importance-sampling formulation. Later, in our Particle Filter, we will use a different method to determine the particle weights.

The Dirac delta $\delta(x-a)$ represents an idealized spike located exactly at $x=a$:

- Here, $\delta(x-x^{(i)})$ is equivalent to the lecture's notation $\delta_{x^{(i)}}(x)$; both represent a Dirac delta located at $x^{(i)}$.
- $\delta(x-a)=0$ everywhere except at $x=a$
- The spike has zero width and unbounded height at $x=a$
- The total area under the spike is exactly $1$:

$$
\int_{-\infty}^{\infty}\delta(x-a)\,dx=1
$$

So $\delta(x-a)$ represents **one unit of probability concentrated at exactly $x=a$**; multiplying it by $w^{(i)}$ gives a point mass with weight $w^{(i)}$.

This means that we approximate the **entire distribution $p(x)$** using these weighted particles, rather than repeatedly calculating the distribution everywhere.

$$
p(x) \approx \sum_{i=1}^{n} w^{(i)}\,\delta(x-x^{(i)})
$$

**Two equivalent views of the same distribution:**
- **PMF form:** just the list of (particle, weight) pairs — $P(X = x^{(i)}) = w^{(i)}$. This is what the particles actually store.
- **Delta-sum form (above):** the same PMF rewritten as a function of continuous $x$, so it can be plugged into the Bayes filter equations, which are written for continuous $p(x)$.

![Sampling Methods](/assets/module-c/lecture-7/sampling-methods.png)

### Problem Setting
A set of $N$ weighted particles represents the posterior:

$$
S = \{\, \langle x^{[i]}, w^{[i]} \rangle \mid i = 1, \dots, N \,\}, \qquad p(x) \approx \sum_{i=1}^{N} w^{(i)}\,\delta_{x^{(i)}}(x)
$$

In localization:
- **Prediction** ← Odometry / Dead Reckoning
- **Correction** ← LiDAR / Scan Matching

### Step 1: Prediction (propagate the dynamics)
Push each particle through the system dynamics with noise:

$$
x_{k+1}^{(i)} = f(x_k^{(i)}, u_k) + \epsilon_k
$$

- $f(x_k, u_k)$: system dynamic function (same role as the action model)
- $x_k^{(i)}$: $i$-th particle at timestep $k$
- $\epsilon_k$: process noise

Equivalent to drawing samples from the proposal distribution:

$$
x_{k+1}^{(i)} \sim p(x_{k+1} \mid x_k, u_k)
$$

### Step 2: Correction (with the observation)
Given a new observation, update each particle's weight using its previous weight and the likelihood of that observation:

**Connection to the Particle Filter:** In localization, the **target** is the posterior distribution we want, while the **proposal** is chosen using the robot's motion model:

$$
q(x_t\mid x_{t-1},u_t)=p(x_t\mid x_{t-1},u_t)
$$

So the PF works as:

$$
\text{predict particles using motion}
\rightarrow
\text{weight using observations}
$$

Using Bayes' rule, the motion-model terms cancel:

$$
w_t^{(i)}
\propto
w_{t-1}^{(i)}
\frac{
p(o_t\mid x_t^{(i)})\,
p(x_t^{(i)}\mid x_{t-1}^{(i)},u_t)
}{
q(x_t^{(i)}\mid x_{t-1}^{(i)},u_t)
}
$$

Therefore,

$$
w_t^{(i)}
\propto
w_{t-1}^{(i)}p(o_t\mid x_t^{(i)})
$$

After **prediction**, each particle represents a possible robot pose. For each particle, we compare the LiDAR scan predicted from that pose with the known map to determine how well that pose agrees with the observation.

$$
S = \frac{\sum_m \sum_n (A_{mn} - \bar{A})(B_{mn} - \bar{B})}{\sqrt{\left(\sum_m \sum_n (A_{mn} - \bar{A})^2\right)\left(\sum_m \sum_n (B_{mn} - \bar{B})^2\right)}}
$$

- $A$: the relevant section of the actual map around the particle's predicted pose
- $B$: occupancy map produced from the particle's predicted scan
- $A_{mn}$, $B_{mn}$: corresponding cells in the two maps
- $\bar{A}$, $\bar{B}$: the mean cell value of each map
- $S$: correlation score; higher $S$ means the scan aligns better with the map

The map section $A$ is chosen based on the particle's **predicted pose**, so the scan and map are compared in the same location.

$$
A_{mn} =
\begin{cases}
1, & \text{wall} \\
0, & \text{free space}
\end{cases}
$$

- Update the particle's weight using its scan score:

In our implementation, the observation likelihood is approximated by the **scan-correlation score**:

$$
S^{(i)}\approx p(o_{k+1}\mid x_{k+1}^{(i)})
$$

So the weight update becomes:

$$
w_{k+1}^{(i)}\leftarrow w_k^{(i)}S^{(i)}
$$

* High $S^{(i)}$ → scan matches the map well → **higher weight**
* Low $S^{(i)}$ → poor match → **lower weight**

![Scan Correlation](/assets/module-c/lecture-7/scan-correlation.png)

The ordinary Bayes Filter does:

$$
\text{previous belief}
\xrightarrow{\text{prediction}}
\text{predicted belief}
\xrightarrow{\text{observation}}
\text{corrected belief}
$$

The Particle Filter does the same thing:

$$
\text{weighted particles}
\xrightarrow{\text{motion model}}
\text{new particles}
\xrightarrow{\text{sensor likelihood}}
\text{new weights}
$$

### Step 3: Normalize the weights

After updating the weights, normalize them so they form a probability distribution:

$$
w_{k+1}^{(i)} \leftarrow
\frac{w_{k+1}^{(i)}}{\sum_{j=1}^{N} w_{k+1}^{(j)}}
$$

Now:

$$
\sum_{i=1}^{N} w_{k+1}^{(i)} = 1
$$

This makes the weights interpretable as the relative probabilities of the particles and allows them to be used for resampling.

### Step 4: Resampling
Without resampling, weight concentrates on a few particles while the rest carry negligible weight (*particle degeneracy*).

Resampling redraws particles in proportion to their weights:
- Start with an initial set of particles drawn from the belief distribution.
- As particles propagate, their weights change and some become dominant.
- High-weight particles are duplicated; low-weight particles die off.
- Resampling draws more particles near high-weight particles, concentrating the cloud around high-probability regions.
- All new particles start with the same weight, allowing the filter to begin the next iteration.

**Result**: the particle cloud efficiently tracks high-probability regions of the posterior.

![Resampling](/assets/module-c/lecture-7/resampling.png)

### Particle Filter Loop (Summary)
1. **Predict**: propagate each particle through the motion model + noise (odometry)
2. **Correct/Weight**: weight each particle by the observation likelihood (scan correlation vs. map)
3. **Normalize**: normalize the particle weights so they sum to 1
4. **Resample**: redraw particles according to their weights

So the full loop is:

$$
\text{Predict}
\rightarrow
\text{Weight}
\rightarrow
\text{Normalize}
\rightarrow
\text{Resample}
\rightarrow
\text{repeat}
$$

## Localization: "Where am I?"
Fuse two noisy information sources to estimate pose:
- **IMU input** (odometry info) → prediction
- **LiDAR input** (observation info) → correction

Neither alone is 100% sure (we always have noise), so the output is a distribution $p(x)$ over pose, refined every prediction/correction cycle.

## Key Takeaways
- Dead reckoning drifts because model + measurement noise accumulates → estimate a distribution, not a point
- Bayes Rule: $\text{posterior} \propto \text{likelihood} \times \text{prior}$ (evidence = normalizer)
- The Markov property + Law of Total Probability turn state estimation into a recursive two-step filter
- Bayes filter = Prediction (action model) → Correction (sensor model), repeated; prediction spreads uncertainty, correction sharpens it
- KF/EKF/UKF assume Gaussian noise for an analytical solution; particle filters use weighted samples for non-Gaussian, complicated distributions
- Particle filter on F1TENTH: predict with odometry, correct via LiDAR scan correlation against the map, then resample
