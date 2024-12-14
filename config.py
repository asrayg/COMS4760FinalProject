import random

"""
This file contains all configurable parameters for the simulation:
- Grid dimensions
- Simulation parameters (timesteps, weather intervals, goal updates)
- Drone parameters (fuel capacity, refueling amounts)
- Traffic update settings
- Visualization settings (figure size, drone colors)
"""

# Grid dimensions
GRID_X = 30
GRID_Y = 30
GRID_Z = 10

# Simulation parameters
TOTAL_TIMESTEPS = 150
WEATHER_CHANGE_INTERVAL_MIN = 4
WEATHER_CHANGE_INTERVAL_MAX = 10
GOAL_UPDATE_TIMESTEPS_COUNT_MIN = 5
GOAL_UPDATE_TIMESTEPS_COUNT_MAX = 10

# Refueling stations
NUM_REFUEL_STATIONS = 100

# Drones
NUM_DRONES = 100
FUEL_CAPACITY_MIN = 70
FUEL_CAPACITY_MAX = 100
REFUEL_AMOUNT_MIN = 40
REFUEL_AMOUNT_MAX = 60

# Traffic
TRAFFIC_UPDATE_INTERVAL = 10
TRAFFIC_UPDATE_COUNT = 10

# Visualization
FIG_SIZE = (18, 14)
DRONE_COLORS = [
    'blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta',
    'brown', 'lime', 'pink', 'teal', 'olive', 'navy', 'maroon',
    'turquoise', 'violet', 'gold', 'silver', 'coral', 'indigo'
]
