#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

# Parameters
FINAL_THETA = 0.0  # radians
MAX_SPEED = 0.7  # meters per second
DIST_THR = 0.15      # meters
ANGLE_THR = 0.05  # radians

class TiagoBaseControl(Node):
    def __init__(self):
        super().__init__('tiago_base_control')
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth_odom',
            self.curr_position_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        self.target = (-0.742589, 0.016805) # goal (x, y)
        self.position = (0.0, 0.0)          # default position value
        self.theta = 0.0                     # default angle value
        self.state = 'rotate'
        self.drifting = 1.0

        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def curr_position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        r = msg.pose.pose.orientation
        self.theta = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))  # radians

    def dest_heading(self):
        x, y = self.position
        return math.atan2(self.target[1] - y, self.target[0] - x)

    def target_distance(self):
        x, y = self.position
        dx = self.target[0] - x
        dy = self.target[1] - y
        return math.sqrt(dx*dx + dy*dy)

    def state_machine_loop(self):
        dist_diff = self.target_distance()
        theta_des = self.dest_heading()
        theta_err = theta_des - self.theta
        theta_err = math.atan2(math.sin(theta_err), math.cos(theta_err))  # normalize [-pi, pi]

        twist = Twist()

        if dist_diff < DIST_THR:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.state = 'final_rotate'
            self.get_logger().info('Arrived at target!')

        if self.state == 'rotate':
            if abs(theta_err) > ANGLE_THR:
                turn_speed = 0.5 * MAX_SPEED
                twist.linear.x = 0.0
                twist.angular.z = turn_speed if theta_err > 0 else -turn_speed
            else:
                self.state = 'forward'

        elif self.state == 'forward':
            self.drifting = 0.4 if dist_diff < 3.5 else 1.0
            if abs(theta_err) > self.drifting:
                self.state = 'rotate'
            else:
                twist.linear.x = 0.5 * MAX_SPEED
                twist.angular.z = 0.0

        elif self.state == 'final_rotate':
            final_err = FINAL_THETA - self.theta
            final_err = math.atan2(math.sin(final_err), math.cos(final_err))
            if abs(final_err) > ANGLE_THR:
                turn_speed = 0.3 * MAX_SPEED
                twist.linear.x = 0.0
                twist.angular.z = turn_speed if final_err > 0 else -turn_speed
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Final rotation done, stopping robot.')
                self.timer.cancel()  # stop the loop

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoBaseControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()