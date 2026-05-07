"""
===============================================================================
ENERGY-OPTIMIZED A* PATH PLANNER
===============================================================================

DESCRIPTION:
This module implements a customized A* pathfinding algorithm designed for 
Automated Guided Vehicles (AGVs). Unlike standard A*, this algorithm integrates 
energy consumption into its cost function. It calculates the energy required for 
each step based on the AGV's current physical state (e.g., driving loaded, 
driving empty) and ensures that the generated path will not drain the battery 
below a critical safety threshold (23%). It also includes helper functions to 
calculate total energy consumption and time for a given sequence of actions.

INPUTS:
- grid (list of lists): A 2D array representing the map (0 = free space, others = obstacles).
- start (tuple): The (x, y) starting coordinates of the AGV.
- goal (tuple): The (x, y) target coordinates.
- battery_level (float): The current battery percentage of the AGV.
- state (string): The current operational state of the AGV (e.g., 'drive_empty').
- state_power_map (dict): A mapping of AGV states to their respective power consumption values.
- state_duration_map (dict): A mapping of AGV states to the time it takes to execute them.
- margin (int, optional): The safety buffer distance to keep away from obstacles.

OUTPUTS:
Returns a tuple containing:
1. path (list of tuples): The sequence of (x, y) coordinates forming the optimal path.
2. remaining_battery (float): The estimated battery percentage upon reaching the goal.
3. actual_steps (int): The total number of grid steps taken in the path.
4. actual_turns (int): The total number of directional changes in the path.
5. final_path_states (list): A list of states corresponding to each step in the path.
===============================================================================
"""
import heapq
import numpy as np
from scipy.interpolate import CubicSpline
import math

# Constants
ENERGY_CAPACITY = 480  # in Wh
BATTERY_THRESHOLD = 23  # %

# A* With Energy Optimization (ROS-ready)
def astar_energy_optimized(grid, start, goal, battery_level, state, state_power_map, state_duration_map, margin=1):
    def heuristic(a, b):
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))  # Chebyshev distance

    def neighbors(grid, node):
        x, y = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            # Check if neighbor is within grid bounds and not an obstacle
            if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]) and grid[ny][nx] == 0:
                yield (nx, ny), dx, dy

    def buffered_is_line_clear(grid, p1, p2, margin_val, steps=50):
        x1, y1 = p1
        x2, y2 = p2
        for t in np.linspace(0, 1, steps):
            x = x1 * (1 - t) + x2 * t
            y = y1 * (1 - t) + y2 * t

            xi, yi = int(round(x)), int(round(y))

            for dx_margin in range(-margin_val, margin_val + 1):
                for dy_margin in range(-margin_val, margin_val + 1):
                    nx, ny = xi + dx_margin, yi + dy_margin
                    
                    if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]):
                        if grid[ny][nx] != 0:
                            return False
        return True

    def is_point_safe(grid, x, y, margin_val):
        xi, yi = int(round(x)), int(round(y))
        for dx_margin in range(-margin_val, margin_val + 1):
            for dy_margin in range(-margin_val, dy_margin + 1): # This was dy_margin + 1 before
                nx, ny = xi + dx_margin, yi + dy_margin
                
                if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]):
                    if grid[ny][nx] != 0:
                        return False
        return True

    # --- TEMPORARY CHANGE: Smoothing and Buffering Disabled ---
    def shortcut_smooth(grid, path, margin_val):
        # For now, simply return the path without any shortcutting or buffering checks
        return path

    def smooth_path(path, grid=None, margin_val=1, nudge_distance=0.1, max_attempts=16):
        # For now, simply return the path without any smoothing or safety checks
        return path
    # --- END TEMPORARY CHANGE ---


    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), 0, start, (0, 0), battery_level))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    actual_steps = 0
    actual_turns = 0
    final_path_states = []

    while open_list:
        current_f, current_g, current, last_dir, current_battery = heapq.heappop(open_list)

        if current == goal:
            path = []
            temp_node = current
            while temp_node in came_from:
                path.append(temp_node)
                temp_node = came_from[temp_node]
            path.append(start)
            path.reverse()

            final_path_states = [state] * len(path) if path else []
            if len(path) > 1:
                actual_steps = len(path) - 1
                for i in range(1, len(path)):
                    px, py = path[i-1]
                    cx, cy = path[i]
                    if (cx - px, cy - py) != last_dir and i > 1:
                        actual_turns += 1
                    last_dir = (cx - px, cy - py)

            return path, current_battery, actual_steps, actual_turns, final_path_states

        for neighbor_coords, dx, dy in neighbors(grid, current):
            energy_cost = calculate_state_energy(state, state_power_map, state_duration_map)
            tentative_g = current_g + energy_cost

            remaining_battery = current_battery - (energy_cost / ENERGY_CAPACITY * 100)

            if remaining_battery < BATTERY_THRESHOLD:
                continue

            if neighbor_coords not in g_score or tentative_g < g_score[neighbor_coords]:
                came_from[neighbor_coords] = current
                g_score[neighbor_coords] = tentative_g
                f_score[neighbor_coords] = tentative_g + heuristic(neighbor_coords, goal)
                heapq.heappush(open_list, (f_score[neighbor_coords], tentative_g, neighbor_coords, (dx, dy), remaining_battery))
                
    return [], battery_level, 0, 0, []

def calculate_state_energy(state, state_power_map, state_duration_map):
    power_components = state_power_map[state]
    duration = state_duration_map[state]
    total_power = sum(power_components)
    return (total_power * duration) / 3600  # Wh

def calculate_total_energy(path_states, state_power_map, state_duration_map):
    total_energy = 0
    for state in path_states:
        power_components = state_power_map[state]
        duration = state_duration_map[state]
        total_power = sum(power_components)
        energy = (total_power * duration) / 3600
        total_energy += energy
    return total_energy

def calculate_total_time(path_states, state_duration_map):
    total_time = sum(state_duration_map[state] for state in path_states)
    return total_time
