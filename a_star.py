import numpy as np
import heapq

class AStar3D:
    """
    A 3D implementation of the A* pathfinding algorithm with dynamic weather and refueling logic.

    Methods:
    --------
    __init__:
        Initializes the A* algorithm with grid, start, goal, wind, fuel, and refueling stations.

    heuristic(a, b):
        Computes the Euclidean distance heuristic between two points in 3D space.

    is_valid(node, occupied):
        Checks if a node is within grid bounds and free from obstacles or occupied nodes.

    get_neighbors(node, occupied):
        Gets all valid neighboring nodes for a given node, considering grid boundaries.

    apply_wind(node, current_wind):
        Applies wind effects to a node and computes the resultant position.

    reconstruct_path(current_state):
        Traces back the path from the goal to the start using the `came_from` dictionary.

    a_star_search(occupied, current_wind):
        Executes the A* algorithm to find a path from start to goal while considering dynamic factors.

    Returns:
        A list of nodes representing the shortest path from start to goal.
    """

    def __init__(self, grid, start, goal, wind_vectors, fuel_capacity, refueling_stations, refuel_amount):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.wind_vectors = wind_vectors
        self.rows, self.cols, self.depth = grid.shape
        self.open_list = []
        self.came_from = {}
        self.g_score = {}
        self.f_score = {}
        self.fuel_capacity = fuel_capacity
        self.refueling_stations = refueling_stations
        self.refuel_amount = refuel_amount

        # Initialize the state with position and fuel
        start_state = (start, fuel_capacity)
        self.g_score[start_state] = 0
        self.f_score[start_state] = self.heuristic(start, goal)
        heapq.heappush(self.open_list, (self.f_score[start_state], start_state))

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def is_valid(self, node, occupied):
        x, y, z = node
        return (
            0 <= x < self.rows and
            0 <= y < self.cols and
            0 <= z < self.depth and
            self.grid[x, y, z] == 0 and
            node not in occupied
        )

    def get_neighbors(self, node, occupied):
        x, y, z = node
        directions = [
            (dx, dy, dz)
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
            for dz in [-1, 0, 1]
            if not (dx == dy == dz == 0)
        ]
        neighbors = [(x + dx, y + dy, z + dz) for dx, dy, dz in directions]
        return [neighbor for neighbor in neighbors if self.is_valid(neighbor, occupied)]

    def apply_wind(self, node, current_wind):
        x, y, z = node
        wind = current_wind.get(node, (0, 0, 0))
        return (
            max(0, min(self.rows - 1, x + wind[0])),
            max(0, min(self.cols - 1, y + wind[1])),
            max(0, min(self.depth - 1, z + wind[2]))
        )

    def reconstruct_path(self, current_state):
        path = []
        while current_state in self.came_from:
            node, _ = current_state
            path.append(node)
            current_state = self.came_from[current_state]
        node, _ = current_state
        path.append(node)
        return path[::-1]

    def a_star_search(self, occupied, current_wind):
        while self.open_list:
            current_f, current_state = heapq.heappop(self.open_list)
            current_node, current_fuel = current_state

            # Check if goal is reached
            if current_node == self.goal:
                return self.reconstruct_path(current_state)

            # Handle refueling
            if current_node in self.refueling_stations and current_fuel < self.fuel_capacity:
                current_fuel = min(current_fuel + self.refuel_amount, self.fuel_capacity)

            for neighbor in self.get_neighbors(current_node, occupied):
                neighbor = self.apply_wind(neighbor, current_wind)
                tentative_g_score = self.g_score[current_state] + 1  # Assuming cost=1 per move

                # Calculate remaining fuel after moving to neighbor
                remaining_fuel = current_fuel - 1
                if remaining_fuel < 0:
                    continue  # Not enough fuel to move

                # If neighbor is a refueling station, consider refueling
                if neighbor in self.refueling_stations:
                    new_fuel = min(remaining_fuel + self.refuel_amount, self.fuel_capacity)
                else:
                    new_fuel = remaining_fuel

                neighbor_state = (neighbor, new_fuel)

                if neighbor_state not in self.g_score or tentative_g_score < self.g_score[neighbor_state]:
                    self.came_from[neighbor_state] = current_state
                    self.g_score[neighbor_state] = tentative_g_score
                    self.f_score[neighbor_state] = tentative_g_score + self.heuristic(neighbor, self.goal)

                    heapq.heappush(self.open_list, (self.f_score[neighbor_state], neighbor_state))

        return []  
