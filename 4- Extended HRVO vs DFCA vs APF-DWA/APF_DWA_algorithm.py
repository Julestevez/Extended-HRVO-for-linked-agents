from math import sqrt, atan2, sin, cos, asin, pi as PI
import numpy as np
import random

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

def APF_DWA_update(X, V_des, V_current, ws_model, pairs):
    """
    Improved Artificial Potential Field with Dynamic Window Approach
    Computes velocities using potential fields and optimizes within dynamic window
    """
    ROB_RAD = ws_model['robot_radius']+0.1
    V_opt = list(V_current)
    
    # Parameters for APF - strengthened and rebalanced
    k_att = 1.5        # Attraction gain (increased)
    k_rep = 20.0       # Repulsion gain (doubled)
    k_pair = 2.0       # Pair cohesion gain (reduced)
    rho_0 = 3.5*ROB_RAD  # Influence distance for repulsion (increased)
    d_pair = 3.5*ROB_RAD  # Desired distance between paired robots
    
    # DWA parameters - expanded window and precision
    window_size = 0.5   # Max velocity change per step (increased)
    samples = 15        # Number of velocity samples to consider (increased)
    
    # Anti-deadlock parameters
    k_random = 0.3     # Random noise to break symmetry
    deadlock_threshold = 0.05  # Velocity threshold to detect deadlock
    
    # Check for deadlock condition globally
    deadlock_detected = False
    active_robots = 0
    
    for i in range(len(X)):
        vel_mag = sqrt(V_current[i][0]**2 + V_current[i][1]**2)
        if vel_mag > deadlock_threshold:
            active_robots += 1
    
    if active_robots < len(X) / 3:  # If less than 1/3 of robots are moving
        deadlock_detected = True
    
    # Assign unique random seeds to each robot to prevent synchronized behavior
    random_seeds = [i * 1000 for i in range(len(X))]
    
    for i in range(len(X)):
        # Set robot-specific random seed for reproducible but diverse behavior
        random.seed(random_seeds[i] + int(X[i][0] * 100 + X[i][1] * 100))
        
        # Current position
        pos_i = X[i]
        
        # Find pair partner if exists
        pair_partner = next((pair[1] for pair in pairs if pair[0] == i), 
                        next((pair[0] for pair in pairs if pair[1] == i), None))
        
        # Goal velocity
        goal_i = V_des[i]
        goal_mag = sqrt(goal_i[0]**2 + goal_i[1]**2)
        
        # 1. COMPUTE ARTIFICIAL POTENTIAL FIELD FORCES
        
        # Attractive force (goal) - with normalization
        if goal_mag > 0.001:
            F_att = [k_att * goal_i[0]/goal_mag, k_att * goal_i[1]/goal_mag]
        else:
            F_att = [0.0, 0.0]
        
        # Initialize repulsive force
        F_rep = [0.0, 0.0]
        
        # Repulsive forces (other robots) - with stronger close-range effect
        for j in range(len(X)):
            if i != j:
                dist_ij = distance(pos_i, X[j])
                
                # Only apply repulsion if within influence distance
                if dist_ij < rho_0:
                    # Vector from robot j to robot i
                    vec_ji = [pos_i[0] - X[j][0], pos_i[1] - X[j][1]]
                    
                    # Normalize
                    if dist_ij > 0.001:
                        vec_ji_norm = [vec_ji[0]/dist_ij, vec_ji[1]/dist_ij]
                        
                        # Enhanced repulsive force with cubic distance falloff
                        rep_mag = k_rep * (1.0/(dist_ij**3) - 1.0/(rho_0**3))
                        
                        # Add to total repulsive force
                        F_rep[0] += rep_mag * vec_ji_norm[0]
                        F_rep[1] += rep_mag * vec_ji_norm[1]
                        
                        # Emergency collision avoidance (extreme close range)
                        if dist_ij < 2.2*ROB_RAD:
                            # Apply very strong repulsion to prevent collision
                            emergency_rep = 5.0 * k_rep * (2.2*ROB_RAD - dist_ij)/(2.2*ROB_RAD)
                            F_rep[0] += emergency_rep * vec_ji_norm[0]
                            F_rep[1] += emergency_rep * vec_ji_norm[1]
        
        # Repulsive forces (links between other pairs)
        for pair in pairs:
            if i not in pair:
                p1, p2 = X[pair[0]], X[pair[1]]
                closest_point = closest_point_on_segment(pos_i, p1, p2)
                dist = distance(pos_i, closest_point)
                
                if dist < rho_0:
                    # Vector from closest point to robot i
                    vec_cp_i = [pos_i[0] - closest_point[0], pos_i[1] - closest_point[1]]
                    
                    # Normalize
                    if dist > 0.001:
                        vec_cp_i_norm = [vec_cp_i[0]/dist, vec_cp_i[1]/dist]
                        
                        # Stronger repulsion from links with cubic distance falloff
                        rep_mag = 2.0 * k_rep * (1.0/(dist**3) - 1.0/(rho_0**3))
                        
                        # Add to total repulsive force
                        F_rep[0] += rep_mag * vec_cp_i_norm[0]
                        F_rep[1] += rep_mag * vec_cp_i_norm[1]
        
        # Initialize pair force
        F_pair = [0.0, 0.0]
        
        # Pair cohesion force (if paired) - with congestion awareness
        if pair_partner is not None:
            pos_partner = X[pair_partner]
            dist_pair = distance(pos_i, pos_partner)
            
            # Vector from robot i to partner
            vec_i_partner = [pos_partner[0] - pos_i[0], pos_partner[1] - pos_i[1]]
            
            if dist_pair > 0.001:
                # Normalize
                vec_i_partner_norm = [vec_i_partner[0]/dist_pair, vec_i_partner[1]/dist_pair]
                
                # Spring-like force (attraction if too far, repulsion if too close)
                pair_mag = k_pair * (dist_pair - d_pair)
                
                # Reduce pair forces in congested areas to allow for navigation
                congestion = 0
                for j in range(len(X)):
                    if j != i and j != pair_partner:
                        d = distance(pos_i, X[j])
                        if d < 3.0*ROB_RAD:
                            congestion += 1
                
                # Scale down pair forces in congestion (allow temporary pair separation)
                if congestion > 2:
                    scaling = max(0.2, 1.0 - (congestion * 0.15))
                    pair_mag *= scaling
                
                # Add to pair force
                F_pair[0] = pair_mag * vec_i_partner_norm[0]
                F_pair[1] = pair_mag * vec_i_partner_norm[1]
        
        # Add random noise component for symmetry breaking
        F_random = [0.0, 0.0]
        
        # Check if this robot is in deadlock
        vel_mag = sqrt(V_current[i][0]**2 + V_current[i][1]**2)
        local_deadlock = vel_mag < deadlock_threshold
        
        # Apply stronger noise if in deadlock
        if local_deadlock or deadlock_detected:
            # Create a random perturbation
            angle = random.uniform(0, 2*PI)
            
            # Noise increases with robot index to create diverse behaviors
            noise_mag = k_random * (1.0 + 0.1 * i)
            
            # Additional boost if robot is at the center of the area
            center_x, center_y = 4.0, 2.5  # Approximate center of the simulation area
            dist_to_center = sqrt((pos_i[0] - center_x)**2 + (pos_i[1] - center_y)**2)
            if dist_to_center < 1.5:
                noise_mag *= 2.0
                
                # Bias the angle to avoid rightward escape
                if angle > PI/4 and angle < 3*PI/4:  # If heading right
                    angle = random.uniform(3*PI/4, 7*PI/4)  # Redirect
            
            F_random = [noise_mag * cos(angle), noise_mag * sin(angle)]
        
        # Total force
        F_total = [F_att[0] + F_rep[0] + F_pair[0] + F_random[0], 
                  F_att[1] + F_rep[1] + F_pair[1] + F_random[1]]
        
        # 2. DYNAMIC WINDOW APPROACH (IMPROVED)
        # Create a set of velocities around current velocity
        v_curr_mag = sqrt(V_current[i][0]**2 + V_current[i][1]**2)
        v_curr_angle = atan2(V_current[i][1], V_current[i][0]) if v_curr_mag > 0.001 else 0.0
        
        # Convert desired force to desired velocity
        v_des_mag = sqrt(F_total[0]**2 + F_total[1]**2)
        v_des_angle = atan2(F_total[1], F_total[0]) if v_des_mag > 0.001 else 0.0
        
        # Limit velocity magnitude
        v_max = sqrt(V_des[i][0]**2 + V_des[i][1]**2) if goal_mag > 0.001 else 1.0
        v_des_mag = min(v_des_mag, v_max)
        
        # Generate candidate velocities within dynamic window with bias toward desired direction
        best_v = V_current[i][:]
        best_cost = float('inf')
        
        # If in congested center, expand search space and velocity range
        in_center = False
        center_x, center_y = 4.0, 2.5  # Approximate center
        if sqrt((pos_i[0] - center_x)**2 + (pos_i[1] - center_y)**2) < 1.5:
            in_center = True
            window_size *= 1.5
            samples += 5
        
        # Generate non-uniform velocity samples focused around desired direction
        for v_sample_idx in range(samples):
            # Generate velocity magnitude samples biased toward higher speeds
            v_sample_factor = (v_sample_idx / (samples-1)) ** 0.7  # Non-linear distribution
            v_sample = max(0.1*v_max, min(v_max, v_curr_mag + (-window_size + 2 * window_size * v_sample_factor)))
            
            # Angular samples with more density near desired direction
            angle_samples = samples + (4 if in_center else 0)  # More angles in center
            for theta_idx in range(angle_samples):
                # Biased angular distribution toward desired direction
                theta_factor = (theta_idx / (angle_samples-1)) * 2 - 1  # -1 to 1
                
                # More focus on the desired angle
                focus_factor = 0.7  # Higher = more focus on desired direction
                theta_factor = theta_factor * (1 - focus_factor) + np.sign(theta_factor) * abs(theta_factor) ** 0.5 * focus_factor
                
                # Calculate the angle sample
                theta_range = PI/2 + (PI/4 if in_center else 0)  # Wider in center
                theta_sample = v_des_angle + theta_factor * theta_range
                
                # Convert to cartesian velocity
                vx = v_sample * cos(theta_sample)
                vy = v_sample * sin(theta_sample)
                
                # Calculate cost (combination of goal distance, obstacle proximity, and heading)
                # Heading cost - prefer moving toward goal
                goal_dir = atan2(goal_i[1], goal_i[0]) if goal_mag > 0.001 else v_des_angle
                angle_diff = abs(((theta_sample - goal_dir + PI) % (2*PI)) - PI)
                cost_heading = angle_diff / PI  # 0 when aligned with goal
                
                # Simulate movement and check minimum distance to obstacles
                look_ahead = 0.75  # Look ahead time in seconds (increased)
                next_pos = [pos_i[0] + vx * look_ahead, pos_i[1] + vy * look_ahead]
                min_dist = float('inf')
                
                # Check distance to other robots with prediction
                for j in range(len(X)):
                    if i != j:
                        # Predict other robot's position
                        next_pos_j = [X[j][0] + V_current[j][0] * look_ahead, 
                                     X[j][1] + V_current[j][1] * look_ahead]
                        dist = distance(next_pos, next_pos_j)
                        min_dist = min(min_dist, dist)
                
                # Check distance to links with prediction
                for pair in pairs:
                    if i not in pair:
                        # Predict pair positions
                        next_p1 = [X[pair[0]][0] + V_current[pair[0]][0] * look_ahead,
                                  X[pair[0]][1] + V_current[pair[0]][1] * look_ahead]
                        next_p2 = [X[pair[1]][0] + V_current[pair[1]][0] * look_ahead,
                                  X[pair[1]][1] + V_current[pair[1]][1] * look_ahead]
                        
                        closest = closest_point_on_segment(next_pos, next_p1, next_p2)
                        dist = distance(next_pos, closest)
                        min_dist = min(min_dist, dist)
                
                # Obstacle cost (higher when close to obstacles)
                cost_obs = 0.0
                if min_dist < rho_0:
                    # Exponential cost increase as distance decreases
                    cost_obs = ((rho_0 - min_dist) / rho_0) ** 2  # Quadratic cost
                    
                    # Emergency cost when very close
                    if min_dist < 2.5*ROB_RAD:
                        cost_obs = 10.0  # Very high cost for near collisions
                
                # Velocity cost (prefer higher velocities except in center)
                cost_vel = 0.0
                if not in_center:
                    cost_vel = (v_max - v_sample) / v_max  # Lower cost for higher velocities
                
                # Path consistency cost (prefer smaller changes from current velocity)
                v_change = sqrt((vx - V_current[i][0])**2 + (vy - V_current[i][1])**2)
                cost_change = v_change / (2 * window_size * v_max)
                
                # Special cost for center navigation - avoid rightward movement in center
                cost_center = 0.0
                if in_center:
                    # Penalize rightward movement to prevent escape to right
                    right_component = max(0, vx)  # Only penalize positive x velocity
                    cost_center = 2.0 * (right_component / v_max) if v_max > 0.001 else 0.0
                
                # Weighted cost function - adjusted weights
                total_cost = (0.4 * cost_heading + 
                             1.2 * cost_obs + 
                             0.2 * cost_vel + 
                             0.3 * cost_change + 
                             0.8 * cost_center)
                
                # Update best velocity if cost is lower
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = [vx, vy]
        
        # Ensure minimum velocity in deadlock situations
        if local_deadlock and best_cost > 2.0:  # If stuck with poor options
            # Generate a random "escape" velocity
            escape_angle = random.uniform(0, 2*PI)
            escape_mag = 0.4 * v_max
            best_v = [escape_mag * cos(escape_angle), escape_mag * sin(escape_angle)]
            
            # Avoid rightward escape in center
            if in_center and best_v[0] > 0:
                best_v[0] *= -0.5  # Reduce rightward component
        
        # Set optimal velocity
        V_opt[i] = best_v
    
    return V_opt

def closest_point_on_segment(p, a, b):
    """
    Find the closest point on segment ab to point p
    """
    ax, ay = a
    bx, by = b
    px, py = p
    
    # Vector from a to b
    abx = bx - ax
    aby = by - ay
    
    # Vector from a to p
    apx = px - ax
    apy = py - ay
    
    # Project ap onto ab, computing the scale factor
    t = (apx * abx + apy * aby) / (abx * abx + aby * aby)
    
    if t < 0:
        return a  # Beyond a
    elif t > 1:
        return b  # Beyond b
    else:
        return [ax + t * abx, ay + t * aby]  # On the segment