from lapy_msg.srv import SaveSpotSrvMsg
from sensor_msgs.msg import Joy

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy


class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_control')
        
        self.spot_count = 0

        self.group_joy = MutuallyExclusiveCallbackGroup()
        self.group_srv_client = MutuallyExclusiveCallbackGroup()
        
        # Joy sub
        self.joy_sub = self.create_subscription(
            Joy, "joy",
            self.joy_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
            callback_group=self.group_joy,
        )

        # All Clients
        self.record_spot_server_client = self.create_client(
            SaveSpotSrvMsg, "record_spot_server", callback_group=self.group_srv_client
        )
        self.record_spot_server_msg = SaveSpotSrvMsg()

    def joy_callback(self, msg: Joy):

        START = msg.buttons[7]

        if START:
            self.record_spot_server_msg.Request.label = f"spot_{self.spot_count}"
            self.record_spot_server_client.call_async(self.record_spot_server_msg)
            self.spot_count += 1
        
        

def main(args=None):
    rclpy.init(args=args)

    joy_control = JoyControl()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(joy_control)
        
    try:
        executor.spin()
    finally:
        executor.shutdown()
        joy_control.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()