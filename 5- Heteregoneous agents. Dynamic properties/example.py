import sys
import os
import numpy as np

from RVO import RVO_update, reach, compute_V_des
from vis import visualize_traj_dynamic

# Create 'data' directory if it doesn't exist
if not os.path.exists('data'):
    os.makedirs('data')

#------------------------------
# Define workspace model
ws_model = dict()
# Robot radius (will be overridden by individual robot radii)
ws_model['robot_radius'] = 0.2
# Circular obstacles, format [x,y,rad]
ws_model['circular_obstacles'] = []
# Rectangular boundary, format [x,y,width/2,height/2]
ws_model['boundary'] = [] 

#------------------------------
# Define robot types/models with different characteristics
robot_types = {
    'heavy_cargo': {
        'mass': 5.0,  # kg
        'radius': 0.25,  # m
        'max_velocity': 0.6,  # m/s
        'max_acceleration': 0.8,  # m/s^2
        'inertia_factor': 0.7,  # Higher inertia, slower response
        'color_offset': 0  # For visualization
    },
    'medium_drone': {
        'mass': 2.5,  # kg
        'radius': 0.2,  # m
        'max_velocity': 1.0,  # m/s
        'max_acceleration': 1.5,  # m/s^2
        'inertia_factor': 0.5,
        'color_offset': 0.25
    },
    'light_scout': {
        'mass': 1.0,  # kg
        'radius': 0.15,  # m
        'max_velocity': 1.5,  # m/s
        'max_acceleration': 2.5,  # m/s^2
        'inertia_factor': 0.3,  # Lower inertia, faster response
        'color_offset': 0.5
    },
    'agile_racer': {
        'mass': 0.5,  # kg
        'radius': 0.1,  # m
        'max_velocity': 2.0,  # m/s
        'max_acceleration': 3.5,  # m/s^2
        'inertia_factor': 0.2,
        'color_offset': 0.75
    }
}

#------------------------------
# Initialization for heterogeneous robots
num_robots = 16
X = [[0.5+2*i, 0.0] for i in range(4)] + [[0.5+2*i+0.5, 0.0] for i in range(4)] + \
    [[0.5+2*i, 5.0] for i in range(4)] + [[0.5+2*i+0.5, 5.0] for i in range(4)]

# Velocity of [vx,vy]
V = [[0,0] for _ in range(len(X))]

# Assign robot types cyclically to create heterogeneous fleet
robot_type_names = list(robot_types.keys())
robot_properties = []
for i in range(num_robots):
    type_name = robot_type_names[i % len(robot_type_names)]
    props = robot_types[type_name].copy()
    props['type'] = type_name
    props['id'] = i
    robot_properties.append(props)

# Extract individual properties for easier access
V_max = [props['max_velocity'] for props in robot_properties]
A_max = [props['max_acceleration'] for props in robot_properties]
masses = [props['mass'] for props in robot_properties]
radii = [props['radius'] for props in robot_properties]
inertia_factors = [props['inertia_factor'] for props in robot_properties]

# Goal positions of [x,y]
goal = [[0.5+2*i, 5.0] for i in range(4)] + [[0.5+2*i+0.5, 5.0] for i in range(4)] + \
       [[0.5+2*i, 0.0] for i in range(4)] + [[0.5+2*i+0.5, 0.0] for i in range(4)]

# Define pairs
pairs = [(i, i+4) for i in range(4)] + [(i+8, i+12) for i in range(4)]

# Store actual velocities (considering inertia)
V_actual = [[0,0] for _ in range(len(X))]

#------------------------------
# Simulation setup
total_time = 20  # Total simulation time (s) - increased for heavier robots
step = 0.01      # Simulation step size

#------------------------------
# Helper function for velocity dynamics with inertia
def update_velocity_with_dynamics(v_current, v_desired, mass, inertia_factor, max_accel, dt):
    """
    Update velocity considering mass, inertia, and acceleration limits
    
    Args:
        v_current: Current velocity [vx, vy]
        v_desired: Desired velocity [vx, vy]
        mass: Robot mass (affects inertia)
        inertia_factor: How much previous velocity affects new velocity (0-1)
        max_accel: Maximum acceleration
        dt: Time step
    
    Returns:
        Updated velocity [vx, vy]
    """
    # Calculate desired acceleration
    a_desired = [(v_desired[i] - v_current[i])/dt for i in range(2)]
    
    # Limit acceleration based on mass and max acceleration
    a_magnitude = np.sqrt(a_desired[0]**2 + a_desired[1]**2)
    if a_magnitude > max_accel:
        scale = max_accel / a_magnitude
        a_desired = [a_desired[i] * scale for i in range(2)]
    
    # Apply inertia effect (heavier robots respond slower)
    v_new = [
        inertia_factor * v_current[i] + (1 - inertia_factor) * (v_current[i] + a_desired[i] * dt)
        for i in range(2)
    ]
    
    return v_new

#------------------------------
# Simulation starts
t = 0
while t*step < total_time:
    # Compute desired velocities to reach goals
    V_des = compute_V_des(X, goal, V_max)
    
    # Compute the optimal velocities to avoid collision
    # Pass robot properties to RVO algorithm
    V = RVO_update(X, V_des, V_actual, ws_model, pairs, robot_properties)
    
    # Update velocities considering dynamics and inertia
    for i in range(len(X)):
        V_actual[i] = update_velocity_with_dynamics(
            V_actual[i], V[i], masses[i], inertia_factors[i], A_max[i], step
        )
    
    # Update positions
    for i in range(len(X)):
        X[i][0] += V_actual[i][0]*step
        X[i][1] += V_actual[i][1]*step
    
    # Visualization
    if t % 10 == 0:
        visualize_traj_dynamic(ws_model, X, V_actual, goal, pairs, 
                             robot_properties=robot_properties,
                             time=t*step, 
                             name=f'data/snap{t//10}.png')
    
    t += 1

print("Simulation complete. Check the 'data' folder for output images.")
print("\nRobot fleet composition:")
for i, props in enumerate(robot_properties):
    print(f"Robot {i}: {props['type']} - mass={props['mass']}kg, "
          f"v_max={props['max_velocity']}m/s, radius={props['radius']}m")