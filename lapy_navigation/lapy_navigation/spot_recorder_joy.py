import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy

import time
import os

package_name = 'lapy_navigation'
package_path = get_package_share_directory(package_name)
spot_list_path = os.path.join(package_path, 'config', 'spot_list.yaml')


class RecordSpotFromJoy(Node):

    def __init__(self):
        super().__init__('record_spot_from_joy')

        self.spot_count = 0

        self.group_joy = MutuallyExclusiveCallbackGroup()
        self.group_amcl = MutuallyExclusiveCallbackGroup()

        # Joy sub
        self.joy_sub = self.create_subscription(
            Joy, "joy",
            self.joy_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
            callback_group=self.group_joy,
        )

        # Amcl sub
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.group_joy
        )

        self.amcl_sub  # prevent unused variable warning

    def pose_callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f %f' % (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w
    
    def joy_callback(self, msg: Joy):

        START = msg.buttons[7]

        if START:
            if hasattr(self, "pos_x"):
                with open(spot_list_path, "a+") as f:
                    f.write(f"\n    spot_{self.spot_count}:")
                    f.write("\n       x : ")
                    f.write(str(self.pos_x))
                    f.write("\n       y : ")
                    f.write(str(self.pos_y))
                    f.write("\n       z : ")
                    f.write(str(self.pos_z))
                    f.write("\n       ox : ")
                    f.write(str(self.ori_x))
                    f.write("\n       oy : ")
                    f.write(str(self.ori_y))
                    f.write("\n       oz : ")
                    f.write(str(self.ori_z))
                    f.write("\n       ow : ")
                    f.write(str(self.ori_w))

                self.get_logger().info(f"Added spot_{self.spot_count} to spot_list.yaml")
                self.spot_count += 1
                time.sleep(1)


def main(args=None):

    rclpy.init(args=args)
    record_spot_from_joy = RecordSpotFromJoy()

    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(record_spot_from_joy)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        record_spot_from_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()