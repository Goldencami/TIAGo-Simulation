#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

# Parameters
FINAL_THETA = 0.0 # radians
MAX_SPEED = 0.7 # meters per second
DIST_THR = 0.10 # used to know when TIAGo is close enough to target
ANGLE_THR = 0.05 # radians
STOP_DIST = 0.45 # meters

class TiagoBaseControl(Node):
    def __init__(self):
        super().__init__('tiago_base_control')

        # subscribe to TIAGo's real position in world
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth_odom',
            self.curr_position_callback,
            10
        )

        # subscribe scan's
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.tiago_scan_callback,
            10
        )

        # subscription to arm movement status
        self.scan_sub = self.create_subscription(
            Bool,
            '/arm_movement_status',
            self.arm_status_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)
        self.arm_move_pub = self.create_publisher(Bool, '/move_arm_command', 10)

        # read position 
        self.position = (0.0, 0.0)
        self.theta = 0.0
        self.state = 'rotate' # forward, rotate, final_rotate
        self.arm_cmd_sent = False
        self.arm_done = False
        self.drifting = 1.0 # value 
        self.target = (-0.600035, 0.013528)  # goal

        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def curr_position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        r = msg.pose.pose.orientation
        self.theta = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y**2 + r.z**2))

    def arm_status_callback(self, msg):
        if msg.data:
            self.arm_done = True
            self.get_logger().info('Arm movement completed.')

    def tiago_scan_callback(self, msg):
        self.scan_data = msg.ranges

    def dest_heading(self):
        x, y = self.position
        return math.atan2(self.target[1] - y, self.target[0] - x)

    def target_distance(self):
        x, y = self.position
        dx = self.target[0] - x
        dy = self.target[1] - y
        return math.sqrt(dx*dx + dy*dy)

    def state_machine_loop(self):
        # measure remaining distance and desired heading
        dist_diff = self.target_distance()
        theta_des = self.dest_heading()
        # compute heading theta error
        theta_err = theta_des - self.theta
        # normalize angle to [-pi, pi]
        theta_err = math.atan2(math.sin(theta_err), math.cos(theta_err))

        twist = Twist() # TIAGo velocity command

        # stop moving and start arm movement to avoid collidng with table
        if dist_diff < STOP_DIST and not self.arm_cmd_sent:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            arm_msg = Bool()
            arm_msg.data = True
            self.arm_move_pub.publish(arm_msg)
            self.arm_cmd_sent = True
            self.arm_done = False
            self.get_logger().info('Arm movement command sent. Waiting...')
            return

        # wait until arm is done
        if self.arm_cmd_sent and not self.arm_done:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # arrived at position, change state to rotate TIAGo
        if dist_diff < DIST_THR:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.state = 'final_rotate'

        # rotate TIAGo to face correct direction when moving towards table
        if self.state == 'rotate':
            # if theta error is significant, rotate while not moving forward
            if abs(theta_err) > ANGLE_THR:
                twist.linear.x = 0.0
                # rotate left or right depending on margin of error
                if theta_err > 0:
                    twist.angular.z = 0.5 * MAX_SPEED
                else:
                    twist.angular.z = -0.5 * MAX_SPEED
            else:
                self.state = 'forward'

        # move TIAGo towards table
        elif self.state == 'forward':
            self.drifting = 0.4 if dist_diff < 3.5 else 1.0 # used for TIAGo to keep moving forward or fix rotation (if arrived to goal)
            if abs(theta_err) > self.drifting:
                self.state = 'rotate'
            else:
                twist.linear.x = 0.5 * MAX_SPEED
                twist.angular.z = 0.0

        # rotate TIAGo to face table
        elif self.state == 'final_rotate':
            # compute error to FINAL_THETA
            final_err = FINAL_THETA - self.theta
            final_err = math.atan2(math.sin(final_err), math.cos(final_err)) # normalize [-pi, pi]
            if abs(final_err) > ANGLE_THR:
                # rotate to final angle
                twist.linear.x = 0.0
                # rotate left or right depending on margin of error
                if final_err > 0:
                    twist.angular.z = 0.3 * MAX_SPEED
                else:
                    twist.angular.z = -0.3 * MAX_SPEED
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Arrived at target! Stopping robot.')
                self.timer.cancel()

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