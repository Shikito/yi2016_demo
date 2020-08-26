import rclpy
from rclpy import Node

# 目標。汎用的なコントローラにしたい

class ControllerManager(Node):
    def __init__(self):
        super().__init__("active_tactile_exploration")