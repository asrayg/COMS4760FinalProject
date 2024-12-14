import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import time

from a_star import AStar3D
from drone import Drone
from utils import update_weather, get_random_free_position, generate_refueling_stations, detect_near_miss
from visualize import visualize_3d

import config

def main():
    """
    Main function to run the drone fleet simulation.
    
    - Initializes a 3D grid representing the environment.
    - Adds obstacles and traffic lanes to the grid.
    - Generates refueling stations and sets up wind vectors.
    - Spawns drones with random starting positions and goals.
    - Updates weather, assigns new goals, and moves drones over timesteps.
    - Detects near-miss collisions and visualizes the simulation in 3D.

    Outputs:
        A real-time 3D visualization of drone movements, goals, and obstacles.
    """

    grid = np.zeros((config.GRID_X, config.GRID_Y, config.GRID_Z))  

    for x in range(5, 25, 5):
        for y in range(5, 25, 5):
            height = random.randint(3, 7)
            grid[x:x+3, y:y+3, 0:height] = 1  

    for x in range(0, 30, 6):
        grid[x, :, 0:10] = 0  

    grid[15, 10, 5] = 0
    grid[20, 15, 4] = 0

    refueling_stations = generate_refueling_stations(grid, config.NUM_REFUEL_STATIONS)

    wind_vectors = {
        (15, 15, 5): (1, 0, 0),
        (16, 15, 5): (0, 1, 0),
        (17, 16, 5): (0, 0, 1)
    }

    drones = []
    for i in range(config.NUM_DRONES):
        fuel_capacity = random.randint(config.FUEL_CAPACITY_MIN, config.FUEL_CAPACITY_MAX)
        refuel_amount = random.randint(config.REFUEL_AMOUNT_MIN, config.REFUEL_AMOUNT_MAX)
        drone = Drone(
            drone_id=i+1,
            grid=grid,
            wind_vectors=wind_vectors,
            fuel_capacity=fuel_capacity,
            refueling_stations=refueling_stations,
            refuel_amount=refuel_amount
        )
        drones.append(drone)

    total_timesteps = config.TOTAL_TIMESTEPS
    weather_change_interval = random.randint(config.WEATHER_CHANGE_INTERVAL_MIN, config.WEATHER_CHANGE_INTERVAL_MAX)

    num_goal_updates = random.randint(config.GOAL_UPDATE_TIMESTEPS_COUNT_MIN, config.GOAL_UPDATE_TIMESTEPS_COUNT_MAX)
    goal_update_timesteps = sorted(random.sample(range(1, total_timesteps + 1), num_goal_updates))
    print(f"Goal updates will occur at timesteps: {goal_update_timesteps}")

    fig = plt.figure(figsize=config.FIG_SIZE)
    ax = fig.add_subplot(111, projection='3d')

    colors = config.DRONE_COLORS

    current_wind = wind_vectors.copy()
    for timestep in range(1, total_timesteps + 1):
        print(f"\n--- Time Step {timestep} ---")

        if timestep % weather_change_interval == 0:
            current_wind = update_weather(current_wind, grid.shape)
            wind_info = ", ".join([f"{k}:{v}" for k, v in current_wind.items()])
            print(f"Weather updated: {wind_info}")

        # Assign new goals at randomized timesteps
        if timestep in goal_update_timesteps:
            print(f"Assigning new goals at timestep {timestep}")
            for drone in drones:
                if drone.status == 'active':
                    new_goal = get_random_free_position(grid, refueling_stations)
                    drone.update_goal(new_goal)

        # Move drones
        for drone in drones:
            if drone.status == 'active':
                drone.move()

        # Detect near-miss collisions
        detect_near_miss(drones, threshold=1.0)

        # Check if all drones have reached their goals
        if all(drone.status == 'reached' for drone in drones):
            print(f"\nAll drones have reached their goals by timestep {timestep}. Ending simulation.")
            # Visualize the final state one last time before breaking
            weather_info = ", ".join([f"{k}:{v}" for k, v in current_wind.items()])
            visualize_3d(ax, grid, drones, refueling_stations, weather_info, timestep, colors)
            plt.pause(0.05)  # Short pause to update the plot
            break

        # Visualize current state
        wind_info = ", ".join([f"{k}:{v}" for k, v in current_wind.items()])
        visualize_3d(ax, grid, drones, refueling_stations, wind_info, timestep, colors)
        plt.pause(0.05)  # Reduced pause for faster animation

        # Introduce dynamic traffic by blocking/unblocking areas
        if timestep % config.TRAFFIC_UPDATE_INTERVAL == 0:
            # Randomly block or unblock some cells
            for _ in range(config.TRAFFIC_UPDATE_COUNT):  # Further increased for more dynamic traffic
                x = random.randint(0, grid.shape[0]-1)
                y = random.randint(0, grid.shape[1]-1)
                z = random.randint(0, grid.shape[2]-1)
                if (x, y, z) not in refueling_stations:
                    grid[x, y, z] = 1 if grid[x, y, z] == 0 else 0
            print(f"Traffic updated at timestep {timestep}.")

    plt.show()

if __name__ == "__main__":
    main()