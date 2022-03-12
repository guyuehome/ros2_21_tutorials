import rclpy
from   rclpy.node import Node     # ROS2 节点类

class ParameterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('robot_name', 'mbot')

    def timer_callback(self):
        robot_name_param = self.get_parameter('robot_name').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % robot_name_param)

        new_name_param = rclpy.parameter.Parameter('robot_name', rclpy.Parameter.Type.STRING, 'mbot')
        all_new_parameters = [new_name_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = ParameterNode('param_declare')
    rclpy.spin(node)
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
