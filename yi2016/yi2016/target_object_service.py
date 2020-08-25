import math

import numpy as np
import rclpy
from rclpy.node import Node

from yi2016_interfaces.srv import Float64MultiArray
from utils.target_object import TargetObject
from utils.multi_array import decode_array, encode_array

class TargetObjectService(Node):

    def __init__(self, target_object):
        super().__init__("target_object_service")
        self.target_object = target_object
        self.srv = self.create_service(
                Float64MultiArray,
                'target_object_surface_shape',
                self.surface_shape_callback
            )

    def surface_shape_callback(self, request, response):
        x = decode_array(request)
        surface_array = self.target_object.surface_shape(x)
        # surface_shape's type is np.array, so it should be transform to list (not nest)
        response = encode_array(surface_array, response)
        return response
        
def main(args=None):
    rclpy.init(args=args)
    target_object = TargetObject()
    target_object_service = TargetObjectService(target_object)
    rclpy.spin(target_object_service)
    rclpy.shutdown()

if __name__=="__main__":
    main()