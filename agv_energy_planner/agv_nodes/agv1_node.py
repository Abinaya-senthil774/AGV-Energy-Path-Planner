"""
===============================================================================
SIMULATED AGV (Automated Guided Vehicle) ROS 2 NODE
===============================================================================

DESCRIPTION:
This script defines a ROS 2 node that simulates an Automated Guided Vehicle (AGV). 
It handles task assignments, executes paths using a custom A* energy-optimized 
routing algorithm, and continuously tracks battery consumption based on physical 
states (e.g., accelerating, driving loaded, picking up, turning). The node 
also features an automatic return-to-charge behavior when battery levels drop 
below a critical threshold.

INPUTS (Subscribes/Receives):
1. Service `/{agv_id}/assign_task` (moiro_interfaces/srv/AssignTask):
   - start (geometry_msgs/Pose): The coordinate where the AGV needs to pick up the load.
   - goal (geometry_msgs/Pose): The coordinate where the AGV needs to drop off the load.
   - urgency (float): A value (e.g., > 0.5 or > 0.7) that influences whether the 
     AGV uses standard driving or accelerated (higher energy) driving modes.
2. Internal Grid: A 10x10 grid map used for pathfinding (currently hardcoded as empty).

OUTPUTS (Publishes/Yields):
1. Topic `/agv_status` (moiro_interfaces/msg/AGVStatus):
   - agv_id (string): The unique identifier of the AGV.
   - battery_level (float): Current battery percentage (0.0 to 100.0%).
   - position (geometry_msgs/Pose): Current X, Y coordinates of the AGV.
   - current_state (string): The current operational state (e.g., 'idle', 'drive_loaded').
2. Standard Logger (Console Out):
   - Step-by-step path execution updates.
   - Total energy used (Wh) and total job time (sec) upon task completion.
   - Warnings and routing information for low battery/charging events.
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from moiro_interfaces.msg import AGVStatus
from moiro_interfaces.srv import AssignTask
import random
import numpy as np
from enum import Enum

from moiro_agv_example.energy_path_planner.energy_path_planner import (
    astar_energy_optimized,
    calculate_state_energy,
    calculate_total_energy,
    calculate_total_time,
    #generate_high_res_grid
)

class AGVTaskState(str, Enum):
    """Enumeration for AGV task states."""
    IDLE = "idle"
    DRIVE_EMPTY = "drive_empty"
    ACCELERATION_EMPTY = "acceleration_empty"
    PICKUP = "pickup"
    DRIVE_LOADED = "drive_loaded"
    ACCELERATION_LOADED = "acceleration_loaded"
    DROP_OFF = "drop_off" # This is correct in the enum
    RETURN_HOME = "return_home"
    STANDBY = "standby"
    TURN = "turn"
    CHARGING = "charging"

def get_state_attributes():
    """Maps AGV task state to power and duration values."""
    STATE_POWER = {
        "drive_empty": [63.19, 55.15, 0],
        "drive_loaded": [63.19, 55.15 + 5.0, 0],
        "acceleration_empty": [63.19, 55.15 + 10, 0],
        "acceleration_loaded": [63.19, 55.15 + 10 + 5.0, 0],
        "deceleration_empty": [63.19, 55.15 + 5, 0],
        "deceleration_loaded": [63.19, 55.15 + 5 + 5.0, 0],
        "pickup": [63.19, 0, 8.67],
        "drop_off": [63.19, 0, 8.67], # FIX: Changed from 'dropoff' to 'drop_off'
        "standby": [7.5, 11.65, 0],
        "turn": [63.19, 55.15, 0],
        "idle": [0, 0, 0],
        "charging": [0, 0, 0]
    }
    STATE_DURATION = {
        "drive_empty": 1.0,
        "drive_loaded": 1.0,
        "acceleration_empty": 1.0,
        "acceleration_loaded": 1.0,
        "deceleration_empty": 1.0,
        "deceleration_loaded": 1.0,
        "pickup": 5.0,
        "drop_off": 5.0, # FIX: Changed from 'dropoff' to 'drop_off'
        "standby": 1.0,
        "turn": 1.0,
        "idle": 0.0,
        "charging": 0.0
    }
    return STATE_POWER, STATE_DURATION

class SimulatedAGVNode(Node):
    def __init__(self, agv_id):
        super().__init__(f'{agv_id}_node')
        self.agv_id = agv_id
        self.home_x = random.randint(0, 9)
        self.home_y = random.randint(0, 9)
        self.battery_level = 23.10
        self.position = Pose()
        self.position.position.x = float(self.home_x)
        self.position.position.y = float(self.home_y)

        self.path = []
        self.path_states = []
        self.path_index = 0
        self.state_history = []
        self.last_task_time = self.get_clock().now()
        self.state = AGVTaskState.STANDBY
        self.task_in_progress = False
        self.has_picked_up = False
        self.task_completed = False
        self.task_states = []

        self.charging_stations = [(1, 0), (9, 1)]

        # --- IMPORTANT CHANGE FOR TESTING ---
        # Instead of a random grid, create an all-clear grid for initial testing.
        base_grid = [[0 for _ in range(10)] for _ in range(10)] # All clear grid
        self.grid = base_grid
        # --- END IMPORTANT CHANGE ---

        self.STATE_POWER, self.STATE_DURATION = get_state_attributes()

        self.status_pub = self.create_publisher(AGVStatus, '/agv_status', 10)
        self.srv = self.create_service(AssignTask, f'/{self.agv_id}/assign_task', self.task_callback)

        self.create_timer(2.0, self.publish_status)
        self.create_timer(1.0, self.execute_next_task_step_timer_callback)

    def find_path(self, start, goal, state):
        try:
            path, remaining_battery, steps, turns, path_states = astar_energy_optimized(
                self.grid, start, goal, self.battery_level, state, self.STATE_POWER, self.STATE_DURATION, margin=0
            )
            if not path:
                raise RuntimeError(f"Path not found for AGV {self.agv_id} from {start} to {goal}.")
            return path, remaining_battery, steps, turns, path_states
        except Exception as e:
            self.get_logger().error(f"Error while finding path: {e}") 
            raise

    def publish_status(self):
        msg = AGVStatus()
        msg.agv_id = self.agv_id
        msg.battery_level = float(self.battery_level)
        msg.position = self.position
        msg.current_state = str(self.state)
        self.status_pub.publish(msg)

        if self.state not in [AGVTaskState.IDLE, AGVTaskState.STANDBY, AGVTaskState.CHARGING] and self.battery_level < 23.0:
            self.get_logger().info(" 🔋 Battery low. Routing to charge...")
            self.go_charge()

    def execute_next_task_step_timer_callback(self): # the function gets called every second to execute the next step in the task
        if not self.task_in_progress:  # 
            now = self.get_clock().now() # get current time
            if (now - self.last_task_time).nanoseconds > 10e9:
                if self.state != AGVTaskState.IDLE:
                    self.state = AGVTaskState.IDLE
                    self.get_logger().info(" 💤 No tasks received in 10 seconds. Entering IDLE state.")
            return

        if self.path_index >= len(self.path_states): 
            if not self.task_completed: # if
                self.finish_task()
            else:
                self.reset_task()
            return

        current_path_state = self.path_states[self.path_index] # it # is the current state in the path_states list wh
        self.state = current_path_state

        if self.path_index < len(self.path): # condition to check if there are more positions in the path, if there 
            pos = self.path[self.path_index]
            self.position.position.x = float(pos[0])
            self.position.position.y = float(pos[1])

        ENERGY_CAPACITY = 480
        energy_used = calculate_state_energy(self.state, self.STATE_POWER, self.STATE_DURATION)
        battery_drain = (energy_used / ENERGY_CAPACITY) * 100.0
        self.battery_level -= battery_drain
        self.battery_level = max(self.battery_level, 0.0)

        self.task_states.append(self.state) 

        self.get_logger().info(
            f" ➡️ Step {self.path_index + 1}/{len(self.path_states)}: {self.state}, "
            f"Position: ({self.position.position.x:.1f}, {self.position.position.y:.1f}), "
            f"Battery: {self.battery_level:.2f}%"
        )
        self.get_logger().info(f" 🔄 Entering state: {self.state}")

        self.path_index += 1 # in

        if self.state == AGVTaskState.CHARGING and self.path_index >= len(self.path_states): # if current state is charging and we have reached the end of the path_states
            self.battery_level = 100.0
            self.state = AGVTaskState.STANDBY
            self.get_logger().info(" ✅ Fully charged. Battery at 100%. Returning to standby.")
            self.reset_task()
            self.task_in_progress = False 
            self.task_completed = True
            return 

    def task_callback(self, request, response): #mistake in logic check
        if self.state not in [AGVTaskState.IDLE, AGVTaskState.STANDBY]:  
            response.accepted = False
            response.reason = "AGV is busy" 
            self.get_logger().warn(f" ❌ Task rejected: {response.reason}")
            return response

        if not hasattr(request, "urgency"):
            self.get_logger().error(" ❌ Urgency value not provided by dispatcher.")
            response.accepted = False
            response.reason = "Urgency missing"
            return response

        self.reset_task() # 

        start = (int(request.start.position.x), int(request.start.position.y))
        goal = (int(request.goal.position.x), int(request.goal.position.y))
        urgency = request.urgency

        try:
            path_to_start, battery1, _, _, states1 = self.find_path(
                (int(self.position.position.x), int(self.position.position.y)),
                start,
                AGVTaskState.DRIVE_EMPTY
            )
        except RuntimeError as e:
            self.get_logger().error(f" ❌ {e}")
            response.accepted = False
            response.reason = "Invalid path to start location"
            self.state = AGVTaskState.STANDBY
            return response

        accel_state_to_goal = (
            AGVTaskState.ACCELERATION_EMPTY
            if urgency > 0.5 and self.battery_level > 50
            else AGVTaskState.DRIVE_EMPTY
        )
        try:
            path_to_goal, battery2, _, _, states2 = self.find_path(
                start,
                goal,
                accel_state_to_goal
            )
        except RuntimeError as e:
            self.get_logger().error(f" ❌ {e}")
            response.accepted = False
            response.reason = "Invalid path to goal location"
            self.state = AGVTaskState.STANDBY
            return response

        pickup_state = [AGVTaskState.PICKUP]

        loaded_accel_state_to_home = (
            AGVTaskState.ACCELERATION_LOADED
            if urgency > 0.7 and self.battery_level > 40
            else AGVTaskState.DRIVE_LOADED
        )
        try:
            path_to_home, battery3, _, _, states3 = self.find_path(
                goal,
                (self.home_x, self.home_y),
                loaded_accel_state_to_home
            )
        except RuntimeError as e:
            self.get_logger().error(f" ❌ {e}")
            response.accepted = False
            response.reason = "Invalid path to return home"
            self.state = AGVTaskState.STANDBY
            return response

        dropoff_state = [AGVTaskState.DROP_OFF]

        self.path = path_to_start + path_to_goal + path_to_home
        self.path_states = (
            states1 +
            [accel_state_to_goal] + # Accel state for path_to_goal start
            pickup_state +
            states2 +
            [loaded_accel_state_to_home] + # Accel state for path_to_home start
            dropoff_state +
            states3
        )

        self.path_index = 0
        self.last_task_time = self.get_clock().now()
        self.state = self.path_states[0] if self.path_states else AGVTaskState.STANDBY
        self.task_in_progress = True
        self.task_completed = False
        self.has_picked_up = False

        response.accepted = True
        response.reason = "Task accepted and initialized"
        self.get_logger().info(f" ✅ Task accepted. Executing path with {len(self.path)} steps.")
        return response

    def finish_task(self): #minor change to move driving empty not given to move to home position
        if not (self.position.position.x == float(self.home_x) and
                self.position.position.y == float(self.home_y)):
            self.position.position.x = float(self.home_x)
            self.position.position.y = float(self.home_y)

        total_energy_used = calculate_total_energy(self.task_states, self.STATE_POWER, self.STATE_DURATION)
        total_time = calculate_total_time(self.task_states, self.STATE_DURATION)
        self.get_logger().info(f" 🏁 Task finished.")
        self.get_logger().info(f" 🔋 Battery remaining: {self.battery_level:.2f}%")
        self.get_logger().info(f" ⚡ Energy used: {total_energy_used:.2f} Wh")
        self.get_logger().info(f" ⏱️ Job time: {total_time:.2f} sec")

        self.state = AGVTaskState.STANDBY
        self.last_task_time = self.get_clock().now()
        self.task_completed = True

    def get_nearest_charging_station(self):
        agv_x, agv_y = int(self.position.position.x), int(self.position.position.y)
        nearest_station = min(
            self.charging_stations,
            key=lambda s: (s[0] - agv_x)*2 + (s[1] - agv_y)*2
        )
        return nearest_station

    def reset_task(self):
        self.task_states = []
        self.path = []
        self.path_states = []
        self.path_index = 0
        self.task_in_progress = False
        self.task_completed = False
        self.has_picked_up = False
        self.get_logger().info(" 🔁 Task state reset.")

    def go_charge(self):
        if self.state == AGVTaskState.CHARGING:
            self.get_logger().info("Already heading to a charging station.")
            return

        target = self.get_nearest_charging_station()
        try:
            self.path, _, _, _, self.path_states = astar_energy_optimized(
                self.grid,
                (int(self.position.position.x), int(self.position.position.y)),
                target,
                self.battery_level,
                AGVTaskState.DRIVE_EMPTY,
                self.STATE_POWER, self.STATE_DURATION, margin=0
            )
            self.path_index = 0
            self.state = "charging"
            self.task_in_progress = True
            self.task_completed = False
            self.get_logger().info(f" 🔌 Routing to charging station at {target}")
        except RuntimeError:
            self.get_logger().warn(" ❌ Could not find path to charging station.")
            self.state = "standby"
            self.task_in_progress = False
            self.task_completed = False


def main(args=None):
    rclpy.init(args=args)
    agv_node = SimulatedAGVNode(agv_id="agv_01")
    try:
        rclpy.spin(agv_node)
    except KeyboardInterrupt:
        agv_node.get_logger().info(" 👋 Shutting down AGV node...")
    finally:
        agv_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
