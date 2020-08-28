import numpy as np
import rclpy
from rclpy.node import Node
import time
import threading

from yi2016_interfaces.msg import ControlState
from yi2016_interfaces.msg import GP
from yi2016_interfaces.srv import Float64MultiArray
from yi2016_utils.node_utils import create_thread
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

        # Publish Control Result
        self.pub_control_state = self.create_publisher(ControlState, 'control_state', 10)
        timer_period = 0.5  # seconds
        self.pub_control_state_timer = self.create_timer(timer_period, self.pub_control_state_callback)
        self.iter_num = 0

        # Publish GP Result
        self.pub_gp_result = self.create_publisher(GP, 'gp_result', 10)
        timer_period = 0.5
        self.pub_gp_result_timer = self.create_timer(timer_period, self.pub_gp_result_callback)
        
        # Main Loop (by Thread)
        update_period = 0.5
        self.control_loop_thread = create_thread(update_period, self.update)
        timer_period = 0.5  # seconds

    # Algorithm 1 Loop in Yi Paper
    def update(self):
        self.get_logger().info('on update')
        
        # 1) 2) 3) in Loop
        x_star = self.controller.calc_next_point()      

        # 4) in Loop # Wait for response
        response = self.request_service_sync(           
            self.cli_target_object, encode_array(array=x_star.x, comm_data=self.req))

        # 5) in Loop
        y_star = decode_array(response)
        self.controller.add_x_y_to_T(x_star.x, y_star)

        self.iter_num += 1

    def pub_gp_result_callback(self):
        if self.isSetController is None:
            return
        x_sampling_points = np.linspace(
            self.controller.bounds[0][0],
            self.controller.bounds[0][1],
            500)  # Number of sampling points
        x_sampling_points = x_sampling_points[:, None] # Reshape For GPy input shape
        msg = GP()
        mean, var = self.controller.model.predict(x_sampling_points)
        train_x = self.controller.X
        train_y = self.controller.Y
        msg.x_sampling_points = encode_array(x_sampling_points, msg.x_sampling_points)
        msg.mean     = encode_array(mean, msg.mean)
        msg.var      = encode_array(var, msg.var)
        msg.train_x  = encode_array(train_x, msg.train_x)
        msg.train_y  = encode_array(train_y, msg.train_y)
        msg.iter_num = self.iter_num 
        self.pub_gp_result.publish(msg)

    def pub_control_state_callback(self):
        msg = ControlState()
        msg.iter_num = self.iter_num
        self.pub_control_state.publish(msg)
        self.get_logger().info(f'Iteration : {msg.iter_num}')

    # Algorithm 1 Loop 4) in Yi Paper
    def obtain_y_at_x(self, x):
        self.req = encode_array(array=x, comm_data=self.req)
        self.future = self.cli_target_object.call_async(self.req)
        while True:
            if not self.isSetController:
                rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Success!')
                    break
        y = decode_array(response)
        return y

    def set_controller(self, controller):
        self.controller = controller
        self.isSetController = True

    def request_service_sync(self, client, req):
        future = client.call_async(req)

        while not future.done():
            self.get_logger().info('waiting for response')
            time.sleep(0.01)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))

        return response


def main(args=None):
    rclpy.init(args=args)

    control_manager = Yi2016ControlManager()

    # Create Initial Training Data
    init_X = np.random.uniform(-2., 2., (2, 1))
    init_Y = control_manager.obtain_y_at_x(init_X)
    initial_points = np.concatenate([init_X, init_Y], axis=1)
    
    # Algorithm 1 Initialize in Yi Paper
    yi_controller = Yi2016Controller(
        input_dim=init_X.shape[-1],
        output_dim=init_Y.shape[-1],
        bounds=[(-3, 3)],
        initial_points=initial_points)
    control_manager.set_controller(yi_controller)

    # Display Modules
    from IPython.display import display
    from matplotlib import pyplot as plt

    control_manager.control_loop_thread.start()
    rclpy.spin(control_manager)

    control_manager.destroy_node()
    rclpy.shutdown()
    
    import ipdb; ipdb.set_trace()



if __name__ == '__main__':
    main()
