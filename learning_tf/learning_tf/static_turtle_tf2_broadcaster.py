import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations


class StaticFramePublisher(Node):
   """
   Broadcast transforms that never change.

   This example publishes transforms from `world` to a static turtle frame.
   The transforms are only published once at startup, and are constant for all
   time.
   """

   def __init__(self, transformation):
      super().__init__('static_turtle_tf2_broadcaster')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms(transformation)

   def make_transforms(self, transformation):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'world'
      static_transformStamped.child_frame_id = sys.argv[1]
      static_transformStamped.transform.translation.x = float(sys.argv[2])
      static_transformStamped.transform.translation.y = float(sys.argv[3])
      static_transformStamped.transform.translation.z = float(sys.argv[4])
      quat = tf_transformations.quaternion_from_euler(
            float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]

      self._tf_publisher.sendTransform(static_transformStamped)


def main():
   logger = rclpy.logging.get_logger('logger')

   # obtain parameters from command line arguments
   if len(sys.argv) < 8:
      logger.info('Invalid number of parameters. Usage: \n'
                  '$ ros2 run learning_tf static_turtle_tf2_broadcaster'
                  'child_frame_name x y z roll pitch yaw')
      sys.exit(0)
   else:
      if sys.argv[1] == 'world':
            logger.info('Your static turtle name cannot be "world"')
            sys.exit(0)

   # pass parameters and initialize node
   rclpy.init()
   node = StaticFramePublisher(sys.argv)
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
