import rclpy
from rclpy import Node

from yi2016_interfaces.srv import Float64MultiArray

# 目標。汎用的なコントローラにしたい

class Yi2016ControlManager(Node):
    def __init__(self):
        super().__init__("yi_2016_control_manager")
        self.cli = self.create_client(
            Float64MultiArray, 'target_object_surface_shape')
