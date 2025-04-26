import sys
import os

from RVO import RVO_update, reach, compute_V_des
from DFCA_algorithm import DFCA_update
from APF_DWA_algorithm import APF_DWA_update
from vis import visualize_traj_dynamic

def run_simulation(algorithm_name):
    """
    Run simulation with the specified algorithm and return statistics
    """
    print(f"Running simulation with {algorithm_name}...")
    
    #------------------------------
    # Define workspace model
    ws_model = dict()
    # Robot radius
    ws_model['robot_radius'] = 0.2
    # Circular obstacles, format [x,y,rad]
    ws_model['circular_obstacles'] = []
    # Rectangular boundary, format [x,y,width/2,height/2]
    ws_model['boundary'] = [] 

    #------------------------------
    # Initialization for robots
    # Position of [x,y]
    X = [[0.5+2*i, 0.0] for i in range(4)] + [[0.5+2*i+0.5, 0.0] for i in range(4)] + \
        [[0.5+2*i, 5.0] for i in range(4)] + [[0.5+2*i+0.5, 5.0] for i in range(4)]

    # Velocity of [vx,vy]
    V = [[0,0] for _ in range(len(X))]

    # Maximal velocity norm
    V_max = [1.0 for _ in range(len(X))]

    # Goal positions of [x,y]
    goal = [[0.5+2*i, 5.0] for i in range(4)] + [[0.5+2*i+0.5, 5.0] for i in range(4)] + \
           [[0.5+2*i, 0.0] for i in range(4)] + [[0.5+2*i+0.5, 0.0] for i in range(4)]

    # Define pairs
    pairs = [(i, i+4) for i in range(4)] + [(i+8, i+12) for i in range(4)]

    #------------------------------
    # Simulation setup
    total_time = 15  # Total simulation time (s)
    step = 0.01      # Simulation step size

    # Create output directory if it doesn't exist
    output_dir = f'data/{algorithm_name}'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    #------------------------------
    # Statistics
    total_dist = [0.0 for _ in range(len(X))]
    min_pair_dist = float('inf')
    min_collision_dist = float('inf')
    completion_time = None

    #------------------------------
    # Simulation starts
    t = 0
    all_completed = False
    
    while t*step < total_time and not all_completed:
        # Compute desired velocities to reach goals
        V_des = compute_V_des(X, goal, V_max)
        
        # Compute the optimal velocities based on algorithm
        if algorithm_name == "RVO":
            V = RVO_update(X, V_des, V, ws_model, pairs)
        elif algorithm_name == "DFCA":
            V = DFCA_update(X, V_des, V, ws_model, pairs)
        elif algorithm_name == "APF_DWA":
            V = APF_DWA_update(X, V_des, V, ws_model, pairs)
        
        # Store old positions for distance calculation
        X_old = [[x[0], x[1]] for x in X]
        
        # Update positions
        for i in range(len(X)):
            X[i][0] += V[i][0]*step
            X[i][1] += V[i][1]*step
            # Calculate distance traveled
            total_dist[i] += distance(X_old[i], X[i])
        
        # Check pair distances
        for pair in pairs:
            pair_dist = distance(X[pair[0]], X[pair[1]])
            min_pair_dist = min(min_pair_dist, pair_dist)
        
        # Check robot-robot distances for collision detection
        for i in range(len(X)):
            for j in range(i+1, len(X)):
                if j != i:
                    collision_dist = distance(X[i], X[j])
                    min_collision_dist = min(min_collision_dist, collision_dist)
        
        # Check if all robots have reached goals
        all_completed = all(reach(X[i], goal[i], 0.1) for i in range(len(X)))
        if all_completed and completion_time is None:
            completion_time = t*step
        
        # Visualization
        if t % 10 == 0:
            visualize_traj_dynamic(ws_model, X, V, goal, pairs, time=t*step, 
                                 name=f'{output_dir}/snap{t//10}.png')
        
        t += 1
    
    # Set completion time to max if not all robots reached goals
    if completion_time is None:
        completion_time = total_time
    
    # Calculate average distance traveled
    avg_dist = sum(total_dist) / len(total_dist)
    
    print(f"Simulation complete for {algorithm_name}")
    print(f"Completion time: {completion_time:.2f}s")
    print(f"Average distance traveled: {avg_dist:.2f}m")
    print(f"Minimum pair distance: {min_pair_dist:.2f}m")
    print(f"Minimum collision distance: {min_collision_dist:.2f}m")
    
    return {
        "algorithm": algorithm_name,
        "completion_time": completion_time,
        "avg_distance": avg_dist,
        "min_pair_dist": min_pair_dist,
        "min_collision_dist": min_collision_dist
    }

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return ((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)**0.5+0.001

# Run simulations with all algorithms
if __name__ == "__main__":
    # Ensure data directory exists
    if not os.path.exists('data'):
        os.makedirs('data')
    
    # List of algorithms to compare
    algorithms = ["RVO", "DFCA", "APF_DWA"]
    
    # Run simulations and collect statistics
    results = []
    for alg in algorithms:
        results.append(run_simulation(alg))
    
    # Print comparison table
    print("\n----- ALGORITHM COMPARISON -----")
    print(f"{'Algorithm':<10} {'Completion':<12} {'Avg Dist':<10} {'Min Pair':<10} {'Min Coll':<10}")
    print(f"{'':^10} {'Time (s)':<12} {'Traveled':<10} {'Distance':<10} {'Distance':<10}")
    print("-" * 55)
    
    for res in results:
        print(f"{res['algorithm']:<10} {res['completion_time']:<12.2f} {res['avg_distance']:<10.2f} "
              f"{res['min_pair_dist']:<10.2f} {res['min_collision_dist']:<10.2f}")
