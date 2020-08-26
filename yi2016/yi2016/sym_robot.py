import rclpy
from rclpy import Node

class SymRobot(Node):
    def __init__(self):
        super().__init__("sym_robot_node")
        