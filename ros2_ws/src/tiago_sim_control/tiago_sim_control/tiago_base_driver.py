#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # message type for commanding robot velocity, defining linear (\(x,y,z\)) and angular (\(x,y,z) as radians/second) movement

FINAL_THETA = 0.0
DIST_THRESHOLD = 0.35
ANGLE_THRESHOLD = 0.15

class TiagoBaseDriver(Node):
    def __init__(self):
        super().__init__('tiago_base_driver')

        self.__robot = webots_node.robot

        # publish base velocity
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # create timer to execute callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.move_forward)

    def move_forward(self):
        velocity = Twist()
        velocity.linear.x = 0.5  # move forward at 0.5 m/s
        self.cmd_vel_pub.publish(velocity)
        self.get_logger().info('Publishing velocity: "%s"' % velocity.linear)

def main(args=None):
    rclpy.init(args=args)
    node = TiagoBaseDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()