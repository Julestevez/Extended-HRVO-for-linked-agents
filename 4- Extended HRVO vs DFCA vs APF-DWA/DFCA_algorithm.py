from math import sqrt, atan2, sin, cos, asin, pi as PI

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

def DFCA_update(X, V_des, V_current, ws_model, pairs):
    """ 
    Distributed Formation Control with Collision Avoidance (DFCA)
    Computes velocities that maintain formation while avoiding collisions
    """
    ROB_RAD = ws_model['robot_radius']+0.1
    V_opt = list(V_current)
    
    # Parameters for the algorithm - tuned for better center congestion handling
    k_goal = 1.2      # Attraction to goal (increased)
    k_form = 1.0      # Formation maintenance (reduced to avoid static behavior)
    k_avoid = 5.0     # Collision avoidance (increased)
    d_safe = 2.5*ROB_RAD  # Safe distance
    d_form = 3.5*ROB_RAD  # Desired formation distance (reduced)
    
    # Anti-deadlock parameters
    k_random = 0.2    # Random noise to break symmetry
    use_noise = False  # Flag to add noise only when needed
    
    # Detect potential deadlock (check if robots are making progress)
    making_progress = False
    for i in range(len(X)):
        vel_mag = sqrt(V_current[i][0]**2 + V_current[i][1]**2)
        if vel_mag > 0.1:  # If any robot is moving with reasonable speed
            making_progress = True
            break
    
    # Enable random noise if deadlock detected (not making progress)
    if not making_progress:
        use_noise = True
    
    for i in range(len(X)):
        # Find pair partner if exists
        pair_partner = next((pair[1] for pair in pairs if pair[0] == i), 
                          next((pair[0] for pair in pairs if pair[1] == i), None))
        
        # Initialize velocity components
        v_goal = [k_goal * V_des[i][0], k_goal * V_des[i][1]]  # Goal attraction
        v_form = [0, 0]  # Formation maintenance
        v_avoid = [0, 0]  # Collision avoidance
        
        # Formation maintenance (only if paired)
        if pair_partner is not None:
            # Vector from robot to partner
            dx = X[pair_partner][0] - X[i][0]
            dy = X[pair_partner][1] - X[i][1]
            dist = distance(X[i], X[pair_partner])
            
            # Compute formation control vector (spring-like)
            if dist > 0.001:  # Avoid division by zero
                # Difference from desired formation distance
                delta_dist = dist - d_form
                # Formation vector - proportional to distance error
                v_form[0] = k_form * delta_dist * dx/dist
                v_form[1] = k_form * delta_dist * dy/dist
                
                # Reduce formation control if robots are in dense areas
                # This allows pairs to separate temporarily to navigate congestion
                congestion = 0
                for j in range(len(X)):
                    if j != i and j != pair_partner:
                        d = distance(X[i], X[j])
                        if d < 3*ROB_RAD:
                            congestion += 1
                
                if congestion > 2:  # If surrounded by multiple robots
                    v_form[0] *= 0.5
                    v_form[1] *= 0.5
        
        # Collision avoidance with all robots - improved with progressive repulsion
        for j in range(len(X)):
            if i != j:
                dx = X[j][0] - X[i][0]
                dy = X[j][1] - X[i][1]
                dist = distance(X[i], X[j])
                
                # Progressive repulsion (stronger at closer distances)
                if dist < d_safe:
                    # Calculate repulsion factor (inverse squared for stronger close-range effect)
                    repulsion = k_avoid * (1.0/(dist**2) - 1.0/(d_safe**2))
                    
                    # Direction vector pointing away from obstacle
                    if dist > 0.001:  # Avoid division by zero
                        v_avoid[0] -= repulsion * dx/dist
                        v_avoid[1] -= repulsion * dy/dist
                    
                    # Apply stronger repulsion if very close (emergency collision avoidance)
                    if dist < 2.2*ROB_RAD:
                        emergency_factor = 2.0
                        v_avoid[0] *= emergency_factor
                        v_avoid[1] *= emergency_factor
        
        # Collision avoidance with links between other pairs
        for pair in pairs:
            if i not in pair:
                p1, p2 = X[pair[0]], X[pair[1]]
                closest_point = closest_point_on_segment(X[i], p1, p2)
                dist = distance(X[i], closest_point)
                
                if dist < d_safe:
                    dx = closest_point[0] - X[i][0]
                    dy = closest_point[1] - X[i][1]
                    
                    # Stronger repulsion from links with progressive factor
                    repulsion = 1.5 * k_avoid * (1.0/(dist**2) - 1.0/(d_safe**2))
                    
                    if dist > 0.001:  # Avoid division by zero
                        v_avoid[0] -= repulsion * dx/dist
                        v_avoid[1] -= repulsion * dy/dist
        
        # Add random noise to break symmetry in case of deadlock
        v_random = [0, 0]
        if use_noise:
            import random
            angle = random.uniform(0, 2*PI)
            v_random = [k_random * cos(angle), k_random * sin(angle)]
            
            # Increase random component if robot is static
            vel_mag = sqrt(V_current[i][0]**2 + V_current[i][1]**2)
            if vel_mag < 0.05:  # If almost static
                v_random[0] *= 3
                v_random[1] *= 3
        
        # Combine all velocity components
        V_opt[i][0] = v_goal[0] + v_form[0] + v_avoid[0] + v_random[0]
        V_opt[i][1] = v_goal[1] + v_form[1] + v_avoid[1] + v_random[1]
        
        # Normalize if exceeding maximum velocity
        v_norm = sqrt(V_opt[i][0]**2 + V_opt[i][1]**2)
        v_max = sqrt(V_des[i][0]**2 + V_des[i][1]**2)
        
        if v_norm > v_max and v_norm > 0.001:
            V_opt[i][0] = V_opt[i][0] * v_max / v_norm
            V_opt[i][1] = V_opt[i][1] * v_max / v_norm
        
        # Ensure minimum velocity in congested areas to prevent getting stuck
        if not making_progress:
            v_norm = sqrt(V_opt[i][0]**2 + V_opt[i][1]**2)
            if v_norm < 0.2 * v_max:  # If moving too slowly
                goal_dir = [V_des[i][0], V_des[i][1]]
                goal_mag = sqrt(goal_dir[0]**2 + goal_dir[1]**2)
                
                if goal_mag > 0.001:  # If goal direction is valid
                    # Scale up to minimum velocity while preserving direction
                    min_vel = 0.2 * v_max
                    scale = min_vel / v_norm if v_norm > 0.001 else 1.0
                    V_opt[i][0] *= scale
                    V_opt[i][1] *= scale
    
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