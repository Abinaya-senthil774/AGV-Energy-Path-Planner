import rclpy
from rclpy.node import Node

class EnergyPathPlannerNode(Node):
    def __init__(self):
        super().__init__('energy_path_planner')
        self.get_logger().info('Energy Path Planner Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = EnergyPathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
