This code resulted in following research article. Use this for citing the code, please:

Estevez J, Manuel Lopez-Guede J, Del Valle-Echavarri J, Caballero-Martin D, Graña M. Enhanced dynamic obstacle avoidance for linked multi-agent systems by an extended hybrid reciprocal velocity obstacle model. Integrated Computer-Aided Engineering. 2025;0(0). doi:10.1177/10692509251368675

# Collision Avoidance Algorithm

This simulation implements a Multi-Agent pathfinding algorithm using **Reciprocal Velocity Obstacles (RVO)**. The logic assumes holonomic robots (point masses) and solves for collision-free velocities in a crowded workspace that includes dynamic agents and static/moving tether links.

The implementation is split into:
*   **Motion Model**: Simple Euler integration.
*   **RVO Geometry**: Constructing velocity obstacles based on neighbors.
*   **Optimization**: A sampling-based solver to find the best velocity.

## 1. Motion Model
The agents move in a 2D plane. For each time step $\Delta t$, the position $P$ of agent $i$ is updated:

$$ P_i(t+\Delta t) = P_i(t) + V_i^{opt} \cdot \Delta t $$

### Desired Velocity ($V_{des}$)
Calculated in `compute_V_des`, the agent attempts to move directly toward its goal $G_i$ at maximum speed $V_{max}$:

$$ V_{des} = V_{max} \cdot \frac{G_i - P_i}{\|G_i - P_i\|} $$

*(Note: If the distance to goal is less than the threshold, $V_{des} \to 0$)*.

---

## 2. Reciprocal Velocity Obstacles (RVO)
The core logic resides in `RVO_update`. Unlike standard Velocity Obstacles (which assume neighbors keep a constant velocity), **Reciprocal** VO assumes that both colliding agents will shift their velocity by half the required amount to avoid collision.

### A. The Velocity Obstacle (VO)
For an agent $A$ and a neighbor $B$, the forbidden velocity region is a cone.

1.  **Relative Position:** $\Delta P = P_B - P_A$
2.  **Combined Radius:** $R_{sum} = R_A + R_B$ (implemented as `2 * robot_radius`).
3.  **Cone Half-Angle ($\alpha$):**
    The angle width of the collision cone is determined by the tangency of the safety radius over the distance:
    $$ \alpha = \arcsin\left(\frac{R_{sum}}{\|\Delta P\|}\right) $$
4.  **Cone Centerline ($\theta$):**
    $$ \theta = \operatorname{atan2}(\Delta P_y, \Delta P_x) $$

The geometric bounds of the cone are $[\theta - \alpha, \theta + \alpha]$.

### B. The Reciprocal Apex
In velocity space, the "tip" (apex) of the collision cone is translated. For RVO, the apex is located at the average of the current velocities:

$$ \text{Apex}_{AB} = \frac{V_A + V_B}{2} $$

*In the code (`RVO.py`), this apex is translated by the robot's current position $P_A$ to allow for geometric intersection checks in the global coordinate frame.*

### C. Tether/Link Avoidance
The simulation defines "pairs" of robots (simulating a physical link). If Agent $A$ is not part of Pair $P (p_1, p_2)$, the link segment $\overline{P_{p1}P_{p2}}$ acts as a linear obstacle.

1.  **Closest Point Calculation:**
    Find the point $C$ on the segment $\overline{P_{p1}P_{p2}}$ that is closest to $P_A$ using vector projection.
2.  **Virtual Obstacle:**
    Treat point $C$ as a static obstacle with radius $R_{robot}$.
3.  **Velocity Obstacle:**
    Since the link "obstacle" is treated as static for the instant calculation:
    $$ \text{Apex}_{link} = 0 \quad (\text{Origin}) $$

---

## 3. Velocity Optimization (Solver)
The function `intersect` determines the optimal velocity $V_{opt}$ using a **sampling-based approach** rather than continuous linear programming.

### Step 1: Sampling
The code generates a grid of candidate velocities ($v_{cand}$) in polar coordinates:
*   $\phi \in [0, 2\pi)$ (Angle)
*   $\rho \in (0, V_{max}]$ (Magnitude)

### Step 2: Intersection Test
A candidate velocity $v_{cand}$ is **admissible** if it lies outside the RVO cones of *all* neighbors and links.

Mathematically, let $v_{rel} = v_{cand} - \text{Apex}$. If the angle of $v_{rel}$ lies within $[\theta - \alpha, \theta + \alpha]$ for any neighbor, the candidate is discarded.

### Step 3: Cost Function
**Case A: Admissible velocities exist**
Choose the candidate closest to the desired velocity vector:
$$ V_{opt} = \operatorname*{argmin}_{v \in \text{admissible}} \| v - V_{des} \| $$

**Case B: No admissible velocities (Crowded)**
If the robot is inevitably going to collide (or violates safety margins), it selects the "safest" bad velocity. It calculates a Time-to-Collision ($tc$) factor and penalizes velocities that cause immediate collisions.

$$ V_{opt} = \operatorname*{argmin}_{v \in \text{inadmissible}} \left( \frac{w}{tc(v)} + \|v - V_{current}\| \right) $$

Where $w$ is a weight factor (set to 0.2 in code) to balance safety vs. tracking the current trajectory.

---

## 4. References

The logic follows the principles of RVO as defined in:

> van den Berg, J., Guy, S. J., Lin, M., & Manocha, D. (2011). **Reciprocal n-body collision avoidance**. *Robotics research*, 3-19.
