#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

# Parameters
MAX_SPEED = 0.7 # meters per second
DIST_THR = 0.10 # used to know when TIAGo is close enough to target
ANGLE_THR = 0.05 # radians
STOP_DIST = 0.50 # meters from table to stop and move arm

class TiagoBaseControl(Node):
    def __init__(self):
        super().__init__('tiago_base_control')

        # subscribe to TIAGo's real position in world
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.curr_position_callback, 10)
        # subscription to arm movement status
        self.arm_status_sub = self.create_subscription(Bool, '/arm_movement_status', self.arm_status_callback, 10)

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

        self.tasks = [
            {'goal': (-0.600035, 0.013528, 0), 'action': 'move_arm_above_table'}, # goal (x, y, yaw (rad))
            {'goal': (5.451085, -2.887168, 0), 'action': 'pick_can'},
            {'goal': (1.0, 1.0, 0), 'action': 'place_can'}, # to decide
            {'goal': (5.451085, -3.305861, 0), 'action': 'pick_cup'},
            {'goal': (1.0, 1.0, 0), 'action': 'place_cup'} # to decide
        ]
        self.current_task_idx = 0
        self.target = self.tasks[self.current_task_idx]['goal']

        # backward motion target
        self.back_target = None

        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def curr_position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        r = msg.pose.pose.orientation
        self.theta = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y**2 + r.z**2))

    def arm_status_callback(self, msg):
        if msg.data:
            self.arm_done = True
            self.get_logger().info('Arm movement completed.')

    # Helper functions
    def dest_heading(self):
        x, y = self.position
        return math.atan2(self.target[1] - y, self.target[0] - x)

    def target_distance(self):
        x, y = self.position
        dx = self.target[0] - x
        dy = self.target[1] - y
        return math.sqrt(dx*dx + dy*dy)

    def send_arm_cmd(self, action):
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_move_pub.publish(arm_msg)
        self.get_logger().info(f"Executing arm action: {action}")
        self.arm_cmd_sent = True
        self.arm_done = False

    def move_back(self, distance=1.0):
        x_target = self.position[0] - distance * math.cos(self.theta)
        y_target = self.position[1] - distance * math.sin(self.theta)

        self.back_target = (x_target, y_target)
        self.state = 'backward'
        self.get_logger().info(f"Moving backward to ({self.back_target[0]:.2f}, {self.back_target[1]:.2f})")


    def state_machine_loop(self):
        if self.current_task_idx >= len(self.tasks):
            self.get_logger().info('Simulation is done!')
            self.timer.cancel()
            return

        twist = Twist() # TIAGo velocity command
        # measure remaining distance and desired heading
        dist_diff = self.target_distance()
        theta_des = self.dest_heading()
        # compute heading theta error
        theta_err = theta_des - self.theta
        # normalize angle to [-pi, pi]
        theta_err = math.atan2(math.sin(theta_err), math.cos(theta_err))

        if self.state == 'backward' and self.back_target is not None:
            dx = self.back_target[0] - self.position[0]
            dy = self.back_target[1] - self.position[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > DIST_THR:
                twist.linear.x = -0.5 * MAX_SPEED  # move backward straight
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = 'rotate'  # resume navigation
                self.back_target = None
                self.get_logger().info('Finished moving backward')

            self.cmd_pub.publish(twist)
            return  # skip other states

        # --------- STATE MACHINE ---------
        # stop moving and start arm movement to avoid colliding with table
        if dist_diff < STOP_DIST and not self.arm_cmd_sent:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.send_arm_cmd(self.tasks[self.current_task_idx]['action'])
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
            self.drifting = 0.4 if dist_diff < 3.5 else 1.0 # allow smooth forward movement
            if abs(theta_err) > self.drifting:
                self.state = 'rotate'
            else:
                twist.linear.x = 0.5 * MAX_SPEED
                twist.angular.z = 0.0

        # rotate TIAGo to face table
        elif self.state == 'final_rotate':
            # compute error to target yaw
            final_err = self.target[2] - self.theta
            final_err = math.atan2(math.sin(final_err), math.cos(final_err)) # normalize [-pi, pi]
            
            if abs(final_err) > ANGLE_THR:
                # rotate left or right depending on margin of error
                if final_err > 0:
                    twist.angular.z = 0.3 * MAX_SPEED
                else:
                    twist.angular.z = -0.3 * MAX_SPEED
            else:
                twist.angular.z = 0.0
                self.get_logger().info(f"Task {self.current_task_idx} completed: {self.tasks[self.current_task_idx]['action']}")
                # move to next task
                self.current_task_idx += 1
                if self.current_task_idx < len(self.tasks):
                    self.target = self.tasks[self.current_task_idx]['goal']
                    # move back to better avoid obstacles when moving to next goal
                    self.move_back(distance=1.0)
                    self.arm_cmd_sent = False
                    self.arm_done = False
                else:
                    self.get_logger().info("All tasks done!")
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