import numpy as np
import rclpy
from rclpy.node import Node

from yi2016_interfaces.msg import ControlState
from yi2016_interfaces.srv import Float64MultiArray
from yi2016_utils.yi_2016_controller import Yi2016Controller
from yi2016_utils.multi_array import encode_array, decode_array

# 目標。汎用的なコントローラにしたい

class Yi2016ControlManager(Node):
    def __init__(self):
        super().__init__("yi_2016_control_manager")
        self.isSetController = False

        # Target Object Surface Preference.
        self.cli_target_object = self.create_client(
            Float64MultiArray, 'target_object_surface_shape')
        while not self.cli_target_object.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Float64MultiArray.Request()

        # Control State Publisher
        self.pub_control_state = self.create_publisher(ControlState, 'control_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_control_state_callback)
        self.iter_num = 0

        # Main Loop
        control_period = 0.5
        self.control_period = self.create_timer(control_period, self.control_loop)

        # Display Result Preference.

    # Algorithm 1 Loop in Yi Paper
    def control_loop(self):
        self.get_logger().info('control_loop')
        x_star = self.controller.calc_next_point()  # Loop 1) 2) 3)
        self.get_logger().info('done calc_next_point')
        self.get_logger().info(str(x_star.x))
        y_star = self.obtain_y_at_x(x_star.x)            # Loop 4)
        self.get_logger().info('done obtain_y_at_x')
        self.controller.add_x_y_to_T(x_star.x, y_star)   # Loop 5)

    def pub_control_state_callback(self):
        msg = ControlState()
        msg.iter_num = self.iter_num
        self.pub_control_state.publish(msg)
        self.get_logger().info(f'Iteration : {msg.iter_num}')

    # Algorithm 1 Loop 4) in Yi Paper
    def obtain_y_at_x(self, x):
        self.req = encode_array(array=x, comm_data=self.req)
        self.future = self.cli_target_object.call_async(self.req)
        self.get_logger().info('waaai')
        while True:
            if not self.isSetController:
                rclpy.spin_once(self)
            if self.future.done():
                try:
                    self.get_logger().info('try')
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Success!')
                    break
        y = decode_array(response)
        self.get_logger().info('get_y')
        return y

    def set_controller(self, controller):
        self.controller = controller
        self.isSetController = True


def main(args=None):
    rclpy.init(args=args)

    control_manager = Yi2016ControlManager()

    # Create Initial Training Data
    init_X = np.random.uniform(-2., 2., (3, 1))
    init_Y = control_manager.obtain_y_at_x(init_X)
    initial_points = np.concatenate([init_X, init_Y], axis=1)
    
    # Algorithm 1 Initialize in Yi Paper
    yi_controller = Yi2016Controller(
        input_dim=init_X.shape[-1],
        output_dim=init_Y.shape[-1],
        bounds=[(-2, 2)],
        initial_points=initial_points)
    control_manager.set_controller(yi_controller)

    # Activate ControlManager
    rclpy.spin(control_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_manager.destroy_node()
    rclpy.shutdown()
    
    import ipdb; ipdb.set_trace()



if __name__ == '__main__':
    main()
