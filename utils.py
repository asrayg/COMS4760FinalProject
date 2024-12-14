import random
import numpy as np  # Add this line


def update_weather(current_wind, grid_shape):
    """
    Updates wind vectors dynamically by modifying random positions within the grid.

    Parameters:
        current_wind (dict): Existing wind vectors in the grid.
        grid_shape (tuple): Dimensions of the grid.

    Returns:
        dict: Updated wind vectors.
    """

    # Randomly change wind vectors
    new_wind = {}
    for _ in range(random.randint(0, 5)):  # Random number of wind changes
        node = (
            random.randint(0, grid_shape[0]-1),
            random.randint(0, grid_shape[1]-1),
            random.randint(0, grid_shape[2]-1)
        )
        wind = (
            random.choice([-1, 0, 1]),
            random.choice([-1, 0, 1]),
            random.choice([-1, 0, 1])
        )
        new_wind[node] = wind
    # Update current wind
    current_wind.update(new_wind)
    return current_wind

def get_random_free_position(grid, refueling_stations):
    """
    Finds a random free position in the grid that is not occupied by obstacles or refueling stations.

    Parameters:
        grid (ndarray): The 3D grid representing the environment.
        refueling_stations (set): Positions of refueling stations.

    Returns:
        tuple: A random free position (x, y, z).
    """

    while True:
        x = random.randint(0, grid.shape[0]-1)
        y = random.randint(0, grid.shape[1]-1)
        z = random.randint(0, grid.shape[2]-1)
        if grid[x, y, z] == 0 and (x, y, z) not in refueling_stations:
            return (x, y, z)

def generate_refueling_stations(grid, num_stations):
    """
    Randomly generates positions for refueling stations within the grid.

    Parameters:
        grid (ndarray): The 3D grid representing the environment.
        num_stations (int): Number of refueling stations to generate.

    Returns:
        set: Positions of refueling stations.
    """

    refuel_stations = set()
    attempts = 0
    max_attempts = num_stations * 10  # To prevent infinite loop
    while len(refuel_stations) < num_stations and attempts < max_attempts:
        x = random.randint(0, grid.shape[0]-1)
        y = random.randint(0, grid.shape[1]-1)
        z = random.randint(0, grid.shape[2]-1)
        if grid[x, y, z] == 0:
            refuel_stations.add((x, y, z))
        attempts += 1
    return refuel_stations

def detect_near_miss(drones, threshold=1.0):
    """
    Detects near-miss events between drones.

    Parameters:
        drones (list): List of active drones in the simulation.
        threshold (float): Distance threshold for near-miss detection.

    Outputs:
        Prints alerts for detected near-miss events.
    """

    active_drones = [drone for drone in drones if drone.status == 'active']
    for i in range(len(active_drones)):
        for j in range(i + 1, len(active_drones)):
            drone1 = active_drones[i]
            drone2 = active_drones[j]
            dist = np.linalg.norm(np.array(drone1.position) - np.array(drone2.position))
            if dist <= threshold and dist > 0:
                print(f"**Near-Miss Alert:** Drone {drone1.drone_id} and Drone {drone2.drone_id} are within {dist:.2f} units.")
