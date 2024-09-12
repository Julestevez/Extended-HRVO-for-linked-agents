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
# Static square blocks, format [x, y, width, height]
ws_model['static_obstacles'] = [
    [1.0, 2.5, 0.25, 0.25],
    [3.0, 2.5, 0.25, 0.25],
    [5.0, 2.5, 0.25, 0.25],
    [7.0, 2.5, 0.25, 0.25]
]

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

#------------------------------
# Simulation starts
t = 0
while t*step < total_time:
    # Compute desired velocities to reach goals
    V_des = compute_V_des(X, goal, V_max)
    
    # Compute the optimal velocities to avoid collision
    V = RVO_update(X, V_des, V, ws_model, pairs)
    
    # Update positions
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    
    # Visualization
    if t % 10 == 0:
        visualize_traj_dynamic(ws_model, X, V, goal, pairs, time=t*step, name=f'data/snap{t//10}.png')
    
    t += 1

print("Simulation complete. Check the 'data' folder for output images.")
