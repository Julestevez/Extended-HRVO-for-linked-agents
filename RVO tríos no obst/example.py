import sys
import os

from RVO import RVO_update, reach, compute_V_des
from vis import visualize_traj_dynamic

# Create 'data' directory if it doesn't exist
if not os.path.exists('data'):
    os.makedirs('data')

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
X = []
for i in range(3):  # 3 groups on each side
    # Bottom groups
    X.extend([
        [1 + 6*i, 0.0],       # Left agent
        [1 + 6*i + 0.5, 0.0], # Middle agent
        [1 + 6*i + 1, 0.0]    # Right agent
    ])
for i in range(3):  # 3 groups on each side
    # Top groups
    X.extend([
        [1 + 6*i, 5.0],       # Left agent
        [1 + 6*i + 0.5, 5.0], # Middle agent
        [1 + 6*i + 1, 5.0]    # Right agent
    ])

# Velocity of [vx,vy]
V = [[0,0] for _ in range(len(X))]

# Maximal velocity norm
V_max = [1.0 for _ in range(len(X))]

# Goal positions of [x,y]
goal = []
for i in range(3):
    # Goals for bottom groups
    goal.extend([
        [1 + 6*i, 5.0],       # Left agent
        [1 + 6*i + 0.5, 5.0], # Middle agent
        [1 + 6*i + 1, 5.0]    # Right agent
    ])
for i in range(3):
    # Goals for top groups
    goal.extend([
        [1 + 6*i, 0.0],       # Left agent
        [1 + 6*i + 0.5, 0.0], # Middle agent
        [1 + 6*i + 1, 0.0]    # Right agent
    ])

# Define groups
groups = []
for i in range(6):  # 6 groups total (3 at bottom, 3 at top)
    group_start = i * 3
    groups.append((group_start, group_start + 1, group_start + 2))

#------------------------------
# Simulation setup
total_time = 20  # Total simulation time (s)
step = 0.01      # Simulation step size

#------------------------------
# Simulation starts
t = 0
while t*step < total_time:
    # Compute desired velocities to reach goals
    V_des = compute_V_des(X, goal, V_max)
    
    # Compute the optimal velocities to avoid collision
    V = RVO_update(X, V_des, V, ws_model, groups)
    
    # Update positions
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    
    # Visualization
    if t % 10 == 0:
        visualize_traj_dynamic(ws_model, X, V, goal, groups, time=t*step, name=f'data/snap{t//10}.png')
    
    t += 1

print("Simulation complete. Check the 'data' folder for output images.")
