import io
import numpy

from matplotlib import pyplot as plt
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from yi2016_interfaces.msg import GP
from yi2016_utils.multi_array import encode_array, decode_array
from sensor_msgs.msg import Image

class DisplayManager(Node):
    def __init__(self):
        super().__init__("display_manager")
        
        self.sub = self.create_subscription(
            GP,
            'gp_result',
            self.draw_gp_result,
            10)
        self.sub # prevent unused variable warning
        self.iter_num = 0

        self.pub_gp_result = self.create_publisher(
            Image,
            "gp_result_graph",
            10
        )
        self.image_msg = None

    def draw_gp_result(self, msg):
        if not msg.iter_num > self.iter_num:
            if self.image_msg is None:
                pass
            else:
                self.pub_gp_result.publish(self.image_msg)
                self.get_logger().info("Published Image")
            return
        
        self.iter_num = msg.iter_num
        x_sampling_points = decode_array(msg.x_sampling_points).T
        mean    = decode_array(msg.mean).T
        var     = decode_array(msg.var).T
        train_x = decode_array(msg.train_x).T
        train_y = decode_array(msg.train_y).T

        # Figure & Axes Setting
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.grid()
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_xlim(
            np.min(x_sampling_points) - 1,
            np.max(x_sampling_points) + 1,)
        ax.set_ylim(-2, 2)

        # Draw Graph
        for x_sampling_points_, mean_, var_, train_x_, train_y_ in zip(
            x_sampling_points,
            mean,
            var,
            train_x,
            train_y):

            plt.plot(x_sampling_points_, mean_)
            ax.fill_between(
                x_sampling_points_, #TODO: 一次元しか対応していない
                mean_ - 3*var_, # 3σを採用
                mean_ + 3*var_,
                facecolor='blue',
                alpha=0.5)
            plt.scatter(train_x_[:-1], train_y_[:-1], c='blue')
            plt.axvline(x=train_x_[-1], c='red')
            if self.iter_num > 2:
                plt.scatter(train_x_[-2], train_y_[-2], c='green')

        # Transform plt to numpy
        buf = io.BytesIO()  # bufferを用意
        plt.savefig(buf, format='png')  # bufferに保持
        enc = np.frombuffer(buf.getvalue(), dtype=np.uint8)  # bufferからの読み出し
        dst = cv2.imdecode(enc, 1)  # デコード
        # dst = dst[:, :, ::-1]  # BGR->RGB
        plt.close()

        # Publish Image
        bridge = CvBridge()
        self.image_msg = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
        try:
            self.pub_gp_result.publish(self.image_msg)
        except CvBridgeError as e:
            print(e)
        self.get_logger().info("Published Image")

def main(args=None):
    rclpy.init()

    display_manager = DisplayManager()

    rclpy.spin(display_manager)

    display_manager.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
        

    
