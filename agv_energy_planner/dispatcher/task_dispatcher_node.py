"""
===============================================================================
SIMULATED TASK DISPATCHER ROS 2 NODE
===============================================================================

DESCRIPTION:
This script defines a ROS 2 node that acts as a central fleet manager or 
Task Dispatcher for multiple Automated Guided Vehicles (AGVs). It continuously 
generates random transport tasks, maintains them in a prioritized queue 
(sorted by urgency), and intelligently assigns them to the most suitable AGV. 
The best AGV is chosen based on a scoring system that weighs the physical 
distance to the task's starting point against the task's urgency. It also 
monitors fleet health, forcing low-battery AGVs to charge instead of taking 
new tasks.

INPUTS (Subscribes/Receives):
1. Topic `/agv_status` (moiro_interfaces/msg/AGVStatus):
   - Receives telemetry and status updates from all active AGVs.
   - Key data used: agv_id, current_state (e.g., idle, standby), battery_level, 
     and position (X, Y coordinates).

OUTPUTS (Service Calls/Yields):
1. Service `/{agv_id}/assign_task` (moiro_interfaces/srv/AssignTask):
   - Acts as a client to send task parameters to a specific AGV.
   - Sends: start (Pose), goal (Pose), and urgency (float).
2. Standard Logger (Console Out):
   - Real-time logging of task queue size, new task generation, AGV status updates, 
     low-battery warnings, and task acceptance/rejection events.
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from moiro_interfaces.msg import AGVStatus
from moiro_interfaces.srv import AssignTask
import math
import random

class TaskDispatcher(Node):
    """Task Dispatcher Node for AGV Management"""
    def __init__(self):
        super().__init__('task_dispatcher')
        self.task_queue = []
        self.agv_statuses = {}

        start_pose = Pose()
        start_pose.position = Point(x=1.0, y=1.0, z=0.0)
        goal_pose = Pose()
        goal_pose.position = Point(x=4.0, y=4.0, z=0.0)
        self.task_queue.append({
            "start": start_pose,
            "goal": goal_pose,
            "urgency": 0.5
        })
        self.get_logger().info("🧾 Added one test task to task_queue")

        self.create_subscription(AGVStatus, '/agv_status', self.status_callback, 10)
        self.create_timer(2.0, self.dispatch_tasks)
        self.create_timer(10.0, self.generate_random_task)

    def status_callback(self, msg):
        self.agv_statuses[msg.agv_id] = msg
        self.get_logger().info(f"[{msg.agv_id}] status: {msg.current_state}, battery: {msg.battery_level:.1f}%")

    def dispatch_tasks(self):
        if not self.task_queue:
            self.get_logger().info("📋 No tasks to dispatch.")
            return

        self.task_queue.sort(key=lambda t: -t["urgency"])

        for task in list(self.task_queue):
            best_agv_id = self.choose_best_agv(task)
            if best_agv_id:
                success = self.send_task_to_agv(best_agv_id, task)
                if success:
                    self.task_queue.remove(task)

        self.get_logger().info(f"📋 Task queue size: {len(self.task_queue)}")

    def choose_best_agv(self, task):
        best_agv = None
        best_score = float('inf')

        for agv_id, status in self.agv_statuses.items():
            if status.current_state == "idle" and status.current_state == "standby":
                self.get_logger().info(f"⏭️ {agv_id} is {status.current_state}, skipping.")
                continue

            if status.battery_level < 23:
                self.get_logger().info(f"🔋 {agv_id} has low battery ({status.battery_level:.1f}%), skipping.")
                self.get_logger().info(f"🔋 Forcing {agv_id} to charging station")
                self.send_charging_task(agv_id)
                continue

            # status.position is a Pose object, task["start"] is a Pose object
            # Pass the .position (which is a Point) to euclidean_distance
            dist = self.euclidean_distance(status.position.position, task["start"].position)
            urgency = float(task["urgency"])
            score = dist - (urgency * 5.0)

            if score < best_score:
                best_score = score
                best_agv = agv_id

        return best_agv

    def send_task_to_agv(self, agv_id, task):
        self.get_logger().info(f"Sending task to {agv_id}")
        client = self.create_client(AssignTask, f'/{agv_id}/assign_task')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{agv_id} assign_task service not available.")
            return False

        req = AssignTask.Request()
        req.start = task["start"]
        req.goal = task["goal"]
        req.urgency = float(task["urgency"])
        future = client.call_async(req)

        def callback(fut):
            try:
                result = fut.result()
                if result.accepted:
                    self.get_logger().info(f"✅ Task accepted by {agv_id}")
                else:
                    self.get_logger().warn(f"❌ {agv_id} rejected the task: {result.reason}")
            except Exception as e:
                self.get_logger().error(f"Exception while getting response: {e}")

        future.add_done_callback(callback)
        return True

    def send_charging_task(self, agv_id):
        self.get_logger().info(f"🔌 Sending {agv_id} to charging station (stub logic)")

    def generate_random_task(self):
        start = Pose()
        goal = Pose()
        start.position = Point(x=float(random.randint(0, 5)), y=float(random.randint(0, 5)), z=0.0)
        goal.position = Point(x=float(random.randint(0, 5)), y=float(random.randint(0, 5)), z=0.0)
        urgency = random.uniform(0.0, 1.0)
        self.task_queue.append({
            "start": start,
            "goal": goal,
            "urgency": urgency
        })
        self.get_logger().info(
            f"📥 New task added: {start.position.x:.1f},{start.position.y:.1f} → "
            f"{goal.position.x:.1f},{goal.position.y:.1f} | urgency={urgency:.2f}"
        )

    def euclidean_distance(self, p1, p2):
        # p1 and p2 are now Point objects due to the fix in choose_best_agv
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx**2 + dy**2)

def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
