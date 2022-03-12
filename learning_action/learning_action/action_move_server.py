import time

from learning_interface.action import MoveCircle

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)

        self._action_server = ActionServer(
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Moving circle...')
        feedback_msg = MoveCircle.Feedback()

        for i in range(0, 360, 30):
            feedback_msg.state = i
            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.state))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = MoveCircle.Result()
        result.finish = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionServer('action_move_server')
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()
