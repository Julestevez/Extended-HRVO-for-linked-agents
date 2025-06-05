#!/usr/bin/env python
import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np

from math import pi as PI
from math import atan2, sin, cos, sqrt

def visualize_traj_dynamic(ws_model, X, U, goal, pairs, robot_properties=None, time=None, name=None):
    figure = pyplot.figure(figsize=(12, 8) if robot_properties else (10, 6))
    ax = figure.add_subplot(1,1,1)
    
    # Get colormap
    cmap = get_cmap(len(X))
    
    # Plot obstacles (if any)
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
                (hole[0]-hole[2], hole[1]-hole[2]),
                2*hole[2], 2*hole[2],
                facecolor= 'red',
                fill = True,
                alpha=1)
        ax.add_patch(srec)
    
    # Plot robots with different sizes and properties if heterogeneous
    for i in range(len(X)):
        if robot_properties:
            radius = robot_properties[i]['radius']
            robot_type = robot_properties[i]['type']
            mass = robot_properties[i]['mass']
            color_offset = robot_properties[i]['color_offset']
            
            # Adjust color based on robot type
            color = cmap(i + color_offset)
            
            # Different edge styles for different robot types
            edge_styles = {
                'heavy_cargo': {'linewidth': 3.0, 'linestyle': '-'},
                'medium_drone': {'linewidth': 2.0, 'linestyle': '-'},
                'light_scout': {'linewidth': 1.5, 'linestyle': '--'},
                'agile_racer': {'linewidth': 1.0, 'linestyle': ':'}
            }
            edge_style = edge_styles.get(robot_type, {'linewidth': 1.0, 'linestyle': '-'})
        else:
            radius = ws_model['robot_radius']
            color = cmap(i)
            edge_style = {'linewidth': 1.0, 'linestyle': '-'}
        
        # Plot robot circle
        robot = matplotlib.patches.Circle(
            (X[i][0], X[i][1]),
            radius=radius,
            facecolor=color,
            edgecolor='black',
            linewidth=edge_style['linewidth'],
            linestyle=edge_style['linestyle'],
            alpha=0.8,
            zorder=2)
        ax.add_patch(robot)
        
        # Plot velocity arrow (scale based on mass if heterogeneous)
        if robot_properties:
            arrow_width = 0.02 + 0.03 * (mass / 5.0)  # Scale arrow width with mass
            arrow_head_width = 0.05 + 0.05 * (mass / 5.0)
            arrow_head_length = 0.1 + 0.05 * (mass / 5.0)
        else:
            arrow_width = 0.02
            arrow_head_width = 0.05
            arrow_head_length = 0.1
            
        if abs(U[i][0]) > 0.01 or abs(U[i][1]) > 0.01:  # Only draw arrow if moving
            ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], 
                    width=arrow_width,
                    head_width=arrow_head_width, 
                    head_length=arrow_head_length, 
                    fc=color, ec=color)
        
        # Add robot ID and type label
        if robot_properties:
            type_abbrev = {
                'heavy_cargo': 'H',
                'medium_drone': 'M', 
                'light_scout': 'L',
                'agile_racer': 'A'
            }
            label = f'{i}{type_abbrev.get(robot_type, "")}'
            ax.text(X[i][0]-0.15, X[i][1]-0.15, label, 
                   fontsize=10, fontweight='bold', zorder=3)
        else:
            ax.text(X[i][0]-0.1, X[i][1]-0.1, str(i), 
                   fontsize=15, fontweight='bold', zorder=3)
        
        # Plot goal with matching color
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=color, 
               markersize=15, linewidth=3.0)
    
    # Plot lines between pairs with varying thickness based on robot masses
    for pair in pairs:
        if robot_properties:
            # Line thickness based on average mass of paired robots
            avg_mass = (robot_properties[pair[0]]['mass'] + robot_properties[pair[1]]['mass']) / 2
            line_width = 0.5 + 1.5 * (avg_mass / 5.0)
        else:
            line_width = 1.0
            
        ax.plot([X[pair[0]][0], X[pair[1]][0]], 
               [X[pair[0]][1], X[pair[1]][1]], 
               'k-', linewidth=line_width, alpha=0.6)
    
    # Add legend for robot types
    if robot_properties:
        # Create legend entries
        legend_elements = []
        unique_types = set(prop['type'] for prop in robot_properties)
        for robot_type in sorted(unique_types):
            props = next(p for p in robot_properties if p['type'] == robot_type)
            color = cmap(props['color_offset'])
            legend_elements.append(
                pyplot.Line2D([0], [0], marker='o', color='w', 
                            label=f"{robot_type}\n(m={props['mass']}kg, v_max={props['max_velocity']}m/s)",
                            markerfacecolor=color, markersize=10)
            )
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1))
    
    if time:
        ax.text(2, 5.5, '$t=%.1f s$' %time, fontsize=20, fontweight='bold')
    
    # Set axes
    ax.set_aspect('equal')
    ax.set_xlim(-1.0, 9.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel(r'$x$ (m)', fontsize=12)
    ax.set_ylabel(r'$y$ (m)', fontsize=12)
    ax.set_title('Multi-Robot Navigation with Heterogeneous Agents', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Adjust layout to prevent legend cutoff
    pyplot.tight_layout()
    
    if name:
        pyplot.savefig(name, dpi=200, bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color