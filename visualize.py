import matplotlib.pyplot as plt
import numpy as np

"""
Visualizes the current state of the simulation in a 3D plot.

Parameters:
    ax (Axes3D): 3D plotting axes.
    grid (ndarray): The 3D grid representing the environment.
    drones (list): List of drones in the simulation.
    refueling_stations (set): Positions of refueling stations.
    weather_info (str): Current weather information to display.
    timestep (int): Current timestep in the simulation.
    colors (list): List of colors for visualizing drones.

Outputs:
    Updates the 3D plot with the current state of the simulation.
"""

def visualize_3d(ax, grid, drones, refueling_stations, weather_info, timestep, colors):
    ax.cla()  # Clear the current axes

    # Plot obstacles
    obstacle_indices = np.where(grid == 1)
    ax.scatter(obstacle_indices[0], obstacle_indices[1], obstacle_indices[2],
               color='gray', alpha=0.5)

    # Plot refueling stations
    if refueling_stations:
        refuel_x, refuel_y, refuel_z = zip(*refueling_stations)
        ax.scatter(refuel_x, refuel_y, refuel_z, color='yellow', s=100, marker='s')

    # Plot drones and their paths
    for i, drone in enumerate(drones):
        color = colors[i % len(colors)]
        
        # Plot traversed path
        if drone.traversed_path:
            path_x, path_y, path_z = zip(*drone.traversed_path)
            ax.plot(path_x, path_y, path_z, color=color, linestyle='--', linewidth=1)

        # Plot drone's current position
        if drone.status == 'active':
            ax.scatter(*drone.position, color=color, s=50, marker='^')
        elif drone.status == 'reached':
            ax.scatter(*drone.position, color=color, s=50, marker='*')

        # Plot start and goal markers
        ax.scatter(*drone.start, color=color, s=100, marker='o')
        ax.scatter(*drone.original_goal, color=color, s=100, marker='^')

    # Display weather and timestep information
    ax.text2D(0.05, 0.95, f"Time Step: {timestep}", transform=ax.transAxes, fontsize=12)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Drone Fleet Simulation with Dynamic Weather and Traffic')
    # Removed ax.legend() to eliminate the legend
    ax.set_xlim(0, grid.shape[0])
    ax.set_ylim(0, grid.shape[1])
    ax.set_zlim(0, grid.shape[2])
    ax.view_init(elev=30, azim=45)  # Set a better viewing angle
    plt.tight_layout()
