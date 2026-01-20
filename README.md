This code resulted in following research article. Use this for citing the code, please:

Estevez J, Manuel Lopez-Guede J, Del Valle-Echavarri J, Caballero-Martin D, Graña M. Enhanced dynamic obstacle avoidance for linked multi-agent systems by an extended hybrid reciprocal velocity obstacle model. *Integrated Computer-Aided Engineering*. 2025;0(0). [doi:10.1177/10692509251368675](https://doi.org/10.1177/10692509251368675)

# Collision Avoidance Algorithm

This simulation implements a **multi-agent pathfinding algorithm** based on  
**Reciprocal Velocity Obstacles (RVO)**. Agents are modeled as *holonomic point masses* navigating a shared workspace containing:

- Dynamic agents  
- Static and moving **tether / link constraints**

The algorithm computes **collision-free velocities** at each timestep.

---

## Algorithm Overview

The implementation is structured into three main components:

- **Motion Model** — Euler integration of agent motion  
- **RVO Geometry** — Construction of reciprocal velocity obstacles  
- **Velocity Optimization** — Sampling-based selection of the optimal velocity  

---

## 1. Motion Model

Agents move in a 2D plane. For agent \( i \), the position update over a timestep \( \Delta t \) is given by:

\[
\mathbf{P}_i(t + \Delta t) = \mathbf{P}_i(t) + \mathbf{V}_i^{\text{opt}} \, \Delta t
\]

where:
- \( \mathbf{P}_i \in \mathbb{R}^2 \) is the agent position  
- \( \mathbf{V}_i^{\text{opt}} \) is the selected collision-free velocity  

---

### Desired Velocity \( \mathbf{V}_{\text{des}} \)

The desired velocity is computed in `compute_V_des`. Each agent attempts to move directly toward its goal \( \mathbf{G}_i \) at maximum speed \( V_{\max} \):

\[
\mathbf{V}_{\text{des}} =
\begin{cases}
V_{\max} \, \dfrac{\mathbf{G}_i - \mathbf{P}_i}{\lVert \mathbf{G}_i - \mathbf{P}_i \rVert}, & \text{if } \lVert \mathbf{G}_i - \mathbf{P}_i \rVert > \varepsilon \\
\mathbf{0}, & \text{otherwise}
\end{cases}
\]

where \( \varepsilon \) is a small threshold indicating goal convergence.

---

## 2. Reciprocal Velocity Obstacles (RVO)

The main collision avoidance logic is implemented in `RVO_update`.

Unlike standard Velocity Obstacles (VO), which assume neighbors maintain constant velocities, **RVO assumes that both agents share responsibility for avoiding collisions**, each contributing half of the velocity change.

---

### A. Velocity Obstacle Construction

For an agent \( A \) and a neighboring agent \( B \):

#### Relative Geometry

\[
\mathbf{d} = \mathbf{P}_B - \mathbf{P}_A
\]

\[
R_{\text{sum}} = R_A + R_B \quad \text{(implemented as } 2 \cdot R_{\text{robot}} \text{)}
\]

#### Cone Half-Angle

The half-angle of the forbidden velocity cone is determined by tangential contact:

\[
\alpha = \arcsin\!\left( \frac{R_{\text{sum}}}{\lVert \mathbf{d} \rVert} \right)
\]

#### Cone Centerline Direction

\[
\theta = \operatorname{atan2}(d_y, d_x)
\]

The forbidden angular region is therefore:

\[
\theta \in [\, \theta - \alpha,\; \theta + \alpha \,]
\]

---

### B. Reciprocal Apex Shift

In velocity space, the apex of the RVO cone is shifted to the **average velocity** of the two agents:

\[
\mathbf{V}_{\text{apex}}^{AB} = \frac{\mathbf{V}_A + \mathbf{V}_B}{2}
\]

In the implementation (`RVO.py`), this apex is translated into the global coordinate frame using the agent’s current position \( \mathbf{P}_A \) to simplify geometric intersection checks.

---

### C. Tether / Link Avoidance

The simulation supports **linked robot pairs**, representing physical tethers.

If agent \( A \) is *not* part of a linked pair \( (p_1, p_2) \), the segment between them is treated as a **linear obstacle**.

#### Closest Point on the Segment

The closest point \( \mathbf{C} \) on the segment \( p_1 \text{--} p_2 \) to \( \mathbf{P}_A \) is computed via vector projection.

#### Virtual Obstacle Model

- \( \mathbf{C} \) is treated as a **static circular obstacle**
- Radius: \( R_{\text{robot}} \)

Since the obstacle is static for the current timestep, its velocity is zero, and the VO apex is:

\[
\mathbf{V}_{\text{apex}}^{\text{link}} = \mathbf{0}
\]

---

## 3. Velocity Optimization (Solver)

The function `intersect` computes the optimal velocity \( \mathbf{V}_{\text{opt}} \) using a **sampling-based approach**.

---

### Step 1: Velocity Sampling

Candidate velocities \( \mathbf{v}_{\text{cand}} \) are generated in polar coordinates:

- Angle: \( \phi \in [0, 2\pi) \)
- Magnitude: \( r \in [0, V_{\max}] \)

---

### Step 2: Intersection Test

For each candidate velocity, define the relative velocity:

\[
\mathbf{v}_{\text{rel}} = \mathbf{v}_{\text{cand}} - \mathbf{V}_{\text{apex}}
\]

A velocity is **inadmissible** if:

\[
\angle(\mathbf{v}_{\text{rel}}) \in [\, \theta - \alpha,\; \theta + \alpha \,]
\]

for *any* neighboring agent or tether obstacle.

---

### Step 3: Cost Function

#### Case A — Admissible Velocities Exist

Select the velocity closest to the desired velocity:

\[
\mathbf{V}_{\text{opt}} =
\arg\min_{\mathbf{v} \in \mathcal{V}_{\text{safe}}}
\left\lVert \mathbf{v} - \mathbf{V}_{\text{des}} \right\rVert
\]

---

#### Case B — No Admissible Velocities (Crowded Scenario)

When all velocities violate safety constraints, the algorithm selects the *least dangerous* option using a **time-to-collision** penalty:

\[
\mathbf{V}_{\text{opt}} =
\arg\min_{\mathbf{v}}
\left(
\frac{w}{\mathrm{tc}(\mathbf{v})}
+
\left\lVert \mathbf{v} - \mathbf{V}_{\text{current}} \right\rVert
\right)
\]

where:
- \( \mathrm{tc}(\mathbf{v}) \) is the estimated time-to-collision  
- \( w \) is a weighting factor (set to **0.2** in the code)

---

## 4. References

> van den Berg, J., Guy, S. J., Lin, M., & Manocha, D. (2011).  
> **Reciprocal n-body collision avoidance**. *Robotics Research*, 3–19.
