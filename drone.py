import random
from a_star import AStar3D
from utils import update_weather, get_random_free_position

class Drone:
    """
    Represents an individual drone navigating in a 3D environment.

    Attributes:
    -----------
    drone_id : int
        Unique identifier for the drone.
    start : tuple
        Starting position of the drone.
    goal : tuple
        Current goal position for the drone.
    position : tuple
        Current position of the drone.
    fuel_capacity : int
        Maximum fuel the drone can hold.
    current_fuel : int
        Remaining fuel in the drone's tank.

    Methods:
    --------
    plan_path():
        Plans a path from the current position to the goal using the A* algorithm.

    move():
        Moves the drone along the planned path, consuming fuel and refueling if necessary.

    assign_new_goal():
        Assigns a new random refueling station as the drone's goal.

    update_goal(new_goal):
        Updates the drone's goal and recalculates the path.
    """

    def __init__(self, drone_id, grid, wind_vectors, fuel_capacity, refueling_stations, refuel_amount):
        self.drone_id = drone_id
        self.start = get_random_free_position(grid, refueling_stations)  # Random start position
        self.original_goal = get_random_free_position(grid, refueling_stations)  # Random goal position
        self.goal = self.original_goal  # Current goal
        self.position = self.start
        self.grid = grid
        self.wind_vectors = wind_vectors.copy()  # Each drone has its own copy of wind vectors
        self.fuel_capacity = fuel_capacity
        self.current_fuel = fuel_capacity
        self.refueling_stations = refueling_stations
        self.refuel_amount = refuel_amount
        self.path = []
        self.path_step = 0
        self.status = 'active'  # 'active' or 'reached'
        self.traversed_path = [self.start]  # Initialize with the starting position

        # Plan initial path
        self.plan_path()

    def plan_path(self):
        astar = AStar3D(
            grid=self.grid,
            start=self.position,
            goal=self.goal,
            wind_vectors=self.wind_vectors,
            fuel_capacity=self.fuel_capacity,
            refueling_stations=self.refueling_stations,
            refuel_amount=self.refuel_amount
        )
        occupied = set()  
        self.path = astar.a_star_search(occupied, self.wind_vectors)
        if not self.path:
            print(f"Drone {self.drone_id}: No path found from {self.position} to {self.goal}. Retrying with updated wind...")
            # Retry pathfinding by updating wind
            self.wind_vectors = update_weather(self.wind_vectors, self.grid.shape)
            self.path = astar.a_star_search(occupied, self.wind_vectors)
            if not self.path:
                print(f"Drone {self.drone_id}: Still no path found. Assigning a new random goal.")
                self.assign_new_goal()
        self.path_step = 0

    def move(self):
        if self.status != 'active' or not self.path or self.path_step >= len(self.path):
            return

        next_position = self.path[self.path_step]
        # Check fuel
        if self.current_fuel <= 0:
            print(f"Drone {self.drone_id} out of fuel! Attempting to find a refueling station.")
            self.assign_new_goal()  # Assign a new goal to reach a refueling station
            return

        # Move to next position
        self.position = next_position
        self.current_fuel -= 1
        self.path_step += 1
        self.traversed_path.append(self.position)

        # Check for refueling
        if self.position in self.refueling_stations:
            self.current_fuel = min(self.current_fuel + self.refuel_amount, self.fuel_capacity)
            print(f"Drone {self.drone_id} refueled at {self.position}.")

        # Check if goal reached
        if self.position == self.goal:
            if self.goal == self.original_goal:
                print(f"Drone {self.drone_id} reached its original goal at {self.position}.")
                self.status = 'reached'
            else:
                print(f"Drone {self.drone_id} reached refueling station at {self.position}. Resuming to original goal.")
                self.goal = self.original_goal
                self.plan_path()

    def assign_new_goal(self):
        # Assign a random refueling station as the new goal
        if self.refueling_stations:
            self.goal = random.choice(list(self.refueling_stations))
            self.plan_path()
            print(f"Drone {self.drone_id} assigned a new goal: {self.goal} for refueling.")

    def update_goal(self, new_goal):
        self.goal = new_goal
        self.original_goal = new_goal  # Update original goal as well
        self.plan_path()
        print(f"Drone {self.drone_id} received a new goal: {self.goal}")