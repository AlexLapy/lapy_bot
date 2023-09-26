from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy


class JoyToPoseActionClient(Node):

    def __init__(self):
        super().__init__('move_to_spot_from_joy',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        
        self.spot_choice = ""
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.joy_sub = self.create_subscription(
            Joy, "joy",
            self.joy_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
            callback_group=self.group1
        )

        self.nav_to_pose_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.group1)


    def joy_callback(self, msg: Joy):

        A = msg.buttons[0]
        B = msg.buttons[1]
        X = msg.buttons[2]
        Y = msg.buttons[3]

        # self.get_logger().info(f"Joy callback with A:{A}, B:{B}, X:{X}, Y:{Y}")
                               
        if A:
            self.spot_choice = "start"
            self.send_goal()
        elif B:
            self.spot_choice = "kitchen"
            self.send_goal()
        elif X:
            self.spot_choice = "r_bed"
            self.send_goal()
        elif Y:
            self.spot_choice = "door"
            self.send_goal()
        
        

    def send_goal(self):

        posex = self.get_parameter_or(self.spot_choice+'.x', None)
        posey = self.get_parameter_or(self.spot_choice+'.y', self.get_parameter_or("start.y"))
        posez = self.get_parameter_or(self.spot_choice+'.z', self.get_parameter_or("start.z"))

        poseox = self.get_parameter_or(self.spot_choice+'.ox', self.get_parameter_or("start.oz"))
        poseoy = self.get_parameter_or(self.spot_choice+'.oy', self.get_parameter_or("start.oy"))
        poseoz = self.get_parameter_or(self.spot_choice+'.oz', self.get_parameter_or("start.oz"))
        poseow = self.get_parameter_or(self.spot_choice+'.ow', self.get_parameter_or("start.ow"))

        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = posex.value
        goal_pose.pose.pose.position.y = posey.value
        goal_pose.pose.pose.position.z = posez.value
        goal_pose.pose.pose.orientation.x = poseox.value
        goal_pose.pose.pose.orientation.y = poseoy.value
        goal_pose.pose.pose.orientation.z = poseoz.value
        goal_pose.pose.pose.orientation.w = poseow.value

        self.nav_to_pose_ac.wait_for_server()

        self._send_goal_future = self.nav_to_pose_ac.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
            
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! ')
        else:
            self.get_logger().info('Navigation failed with status: {0}'.format(status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(f"Current pose = {feedback.current_pose}")
        self.get_logger().info(f"Navigation time = {feedback.navigation_time}")
        self.get_logger().info(f"Estimated time remaining = {feedback.estimated_time_remaining}")
        self.get_logger().info(f"Number of recoveries = {feedback.number_of_recoveries}")
        self.get_logger().info(f"Distance remaining = {feedback.distance_remaining}")

def main(args=None):
    rclpy.init(args=args)

    action_client = JoyToPoseActionClient()

    # TODO Might want to revert back to single thread
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(action_client)
        
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_client.destroy_node()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()