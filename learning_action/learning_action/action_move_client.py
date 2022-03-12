import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from learning_interface.action import MoveCircle

class MoveCircleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self._action_client = ActionClient(self, MoveCircle, 'move_circle')

    def send_goal(self, enable):
        goal_msg = MoveCircle.Goal()
        goal_msg.enable = enable

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Result: {0}'.format(result.finish))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))


def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionClient('action_move_client')
    node.send_goal(True)
    rclpy.spin(node)
