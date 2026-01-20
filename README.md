This code resulted in following research article. Use this for citing the code, please:

Estevez J, Manuel Lopez-Guede J, Del Valle-Echavarri J, Caballero-Martin D, Graña M. Enhanced dynamic obstacle avoidance for linked multi-agent systems by an extended hybrid reciprocal velocity obstacle model. *Integrated Computer-Aided Engineering*. 2025;0(0). [doi:10.1177/10692509251368675](https://doi.org/10.1177/10692509251368675)

# Collision Avoidance Algorithm

This simulation implements a Multi-Agent pathfinding algorithm using **Reciprocal Velocity Obstacles (RVO)**. The logic assumes holonomic robots (point masses) and solves for collision-free velocities in a crowded workspace that includes dynamic agents and static/moving tether links.

The implementation is split into:
*   **Motion Model**: Simple Euler integration.
*   **RVO Geometry**: Constructing velocity obstacles based on neighbors.
*   **Optimization**: A sampling-based solver to find the best velocity.

## 1. Motion Model
The agents move in a 2D plane. For each time step `dt`, the position `P` of agent `i` is updated:

    P_i(t + dt) = P_i(t) + V_i_opt * dt

### Desired Velocity (V_des)
Calculated in `compute_V_des`, the agent attempts to move directly toward its goal `G_i` at maximum speed `V_max`:

    V_des = V_max * (G_i - P_i) / ||G_i - P_i||

*(Note: If the distance to goal is less than the threshold, V_des → 0)*

---

## 2. Reciprocal Velocity Obstacles (RVO)
The core logic resides in `RVO_update`. Unlike standard Velocity Obstacles (which assume neighbors keep a constant velocity), **Reciprocal** VO assumes that both colliding agents will shift their velocity by half the required amount to avoid collision.

### A. The Velocity Obstacle (VO)
For an agent `A` and a neighbor `B`, the forbidden velocity region is a cone.

1.  **Relative Position:** `diff_P = P_B - P_A`
2.  **Combined Radius:** `R_sum = R_A + R_B` (implemented as `2 * robot_radius`).
3.  **Cone Half-Angle (alpha):**
    The angle width of the collision cone is determined by the tangency of the safety radius over the distance:

    alpha = asin( R_sum / ||diff_P|| )

4.  **Cone Centerline (theta):**

    theta = atan2(diff_P.y, diff_P.x)

The geometric bounds of the cone are `[theta - alpha, theta + alpha]`.

### B. The Reciprocal Apex
In velocity space, the "tip" (apex) of the collision cone is translated. For RVO, the apex is located at the average of the current velocities:

    Apex_AB = (V_A + V_B) / 2

*In the code (`RVO.py`), this apex is translated by the robot's current position `P_A` to allow for geometric intersection checks in the global coordinate frame.*

### C. Tether/Link Avoidance
The simulation defines "pairs" of robots (simulating a physical link). If Agent `A` is not part of Pair `P (p1, p2)`, the link segment `p1-p2` acts as a linear obstacle.

1.  **Closest Point Calculation:**
    Find the point `C` on the segment `p1-p2` that is closest to `P_A` using vector projection.
2.  **Virtual Obstacle:**
    Treat point `C` as a static obstacle with radius `R_robot`.
3.  **Velocity Obstacle:**
    Since the link "obstacle" is treated as static for the instant calculation:

    Apex_link = 0 (Origin)

---

## 3. Velocity Optimization (Solver)
The function `intersect` determines the optimal velocity `V_opt` using a **sampling-based approach** rather than continuous linear programming.

### Step 1: Sampling
The code generates a grid of candidate velocities (`v_cand`) in polar coordinates:
*   **Angle:** 0 to 2π
*   **Magnitude:** 0 to V_max

### Step 2: Intersection Test
A candidate velocity `v_cand` is **admissible** if it lies outside the RVO cones of *all* neighbors and links.

Mathematically, let `v_rel = v_cand - Apex`. If the angle of `v_rel` lies within `[theta - alpha, theta + alpha]` for any neighbor, the candidate is discarded.

### Step 3: Cost Function
**Case A: Admissible velocities exist**
Choose the candidate closest to the desired velocity vector:

    V_opt = argmin( || v - V_des || )

**Case B: No admissible velocities (Crowded)**
If the robot is inevitably going to collide (or violates safety margins), it selects the "safest" bad velocity. It calculates a Time-to-Collision (`tc`) factor and penalizes velocities that cause immediate collisions.

    V_opt = argmin( (w / tc(v)) + ||v - V_current|| )

Where `w` is a weight factor (set to 0.2 in code) to balance safety vs. tracking the current trajectory.

---

## 4. References

The logic follows the principles of RVO as defined in:

> van den Berg, J., Guy, S. J., Lin, M., & Manocha, D. (2011). **Reciprocal n-body collision avoidance**. *Robotics research*, 3-19.
