#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math

# Parameters
MAX_SPEED = 0.7
GOAL_THR = 0.10 # marging error of distance goal threshold
ANGLE_THR = 0.05 # marging error of angle when navigating (used to know if TIAGo needs to fix direction)
LIFT_DIST = 0.70 # used to stop TIAGo and retract arm

class TiagoBaseControl(Node):
    def __init__(self):
        super().__init__('tiago_base_control')
        # subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth_odom', self.curr_position_callback, 10)
        # publishers
        self.cmd_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        # clients
        self.lift_arm_client = self.create_client(Trigger, 'lift_arm')
        self.grab_pose_client = self.create_client(Trigger, 'grab_pose')
        self.pick_obj_client = self.create_client(Trigger, 'pick_obj')
        self.place_obj_client = self.create_client(Trigger, 'place_obj')

        # pending requests
        self.lift_future = None
        self.grab_pose_future = None
        self.pick_obj_future = None
        self.place_future = None


        # TIAGo's initial position and angle
        self.position = (0.0, 0.0)
        self.theta = 0.0
        # TIAGo's states in SM
        self.state = 'ROTATE' # ROTATE, FORWARD, POSE_ARM, FIX_ANGLE, PICKUP, PLACE_OBJ, BACKWARD, RETRACT_ARM, END

        self.tasks = [
            {'goal': (-0.600035, 0.013528, 0), 'action': 'move_arm_above_table'}, # goal (x, y, yaw (rad))
            {'goal': (5.654795, -2.571507, 0), 'action': 'pick_can'},
            {'goal': (6.628535, -2.607165, -1.575030), 'action': 'place_can'},
            {'goal': (5.654795, -2.571507, 0), 'action': 'pick_cup'},
            {'goal': (6.366570, -1.895496, -1.575030), 'action': 'place_cup'}
        ]
        self.current_task_idx = 1 # set to 0 after done testing
        self.target = self.tasks[self.current_task_idx]['goal']
        # new target position when going backwards
        self.backTargetSet = False

        # transition states variables
        self.isArmPosed = False
        self.isObjectPlaced = False
        self.isObjectPicked = False

        self.timer = self.create_timer(0.1, self.state_machine_loop)


    def curr_position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        r = msg.pose.pose.orientation
        self.theta = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y**2 + r.z**2))

    # Helper functions for navigation
    def dest_heading(self):
        x, y = self.position
        return math.atan2(self.target[1] - y, self.target[0] - x)

    def target_distance(self):
        x, y = self.position
        dx = self.target[0] - x
        dy = self.target[1] - y
        return math.sqrt(dx*dx + dy*dy)

    # helper request functions for state machines
    def lift_arm_request(self):
        if self.lift_future:
            if self.lift_future.done():
                result = self.lift_future.result()
                if result.success:
                    self.isArmPosed = True
                self.lift_future = None
            return

        if not self.lift_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for lift_arm service...')
            return

        req = Trigger.Request()
        self.lift_future = self.lift_arm_client.call_async(req)

    def grab_pose_request(self):
        if self.grab_pose_future:
            if self.grab_pose_future.done():
                result = self.grab_pose_future.result()
                if result.success:
                    self.isArmPosed = True
                    self.state = 'ROTATE'
                self.grab_pose_future = None
            return

        if not self.grab_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for grab_pose service...')
            return

        req = Trigger.Request()
        self.grab_pose_future = self.grab_pose_client.call_async(req)

    def pick_obj_request(self):
        if self.pick_obj_future:
            if self.pick_obj_future.done():
                result = self.pick_obj_future.result()
                if result.success:
                    self.isArmPosed = False
                    self.isObjectPicked = True
                    self.state = 'BACKWARD'
                self.pick_obj_future = None
            return

        if not self.pick_obj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for pick_obj service...')
            return

        req = Trigger.Request()
        self.pick_obj_future = self.pick_obj_client.call_async(req)

    def place_obj_request(self):
        if self.place_future:
            if self.place_future.done():
                result = self.place_future.result()
                if result.success:
                    self.isObjectPlaced = True
                    self.state = 'ROTATE'
                self.place_future = None
            return

        if not self.place_obj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for place_obj service...')
            return

        req = Trigger.Request()
        self.place_future = self.place_obj_client.call_async(req)

    # helper function for states
    def set_move_back_to(self, distance=1.0):
        x_target = self.position[0] - distance * math.cos(self.theta)
        y_target = self.position[1] - distance * math.sin(self.theta)

        # self.backTargetSet = (x_target, y_target)
        self.target = (x_target, y_target, self.target[2])
        self.backTargetSet = True
        self.get_logger().info(f"Moving backward to ({self.target[0]:.2f}, {self.target[1]:.2f})")

    def state_machine_loop(self):
        if self.current_task_idx >= len(self.tasks):
            self.state = 'END'
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

        # rotate TIAGo to face correct direction when moving towards table
        if self.state == 'ROTATE':
            # if theta error is significant, rotate while not moving forward
            if abs(theta_err) > ANGLE_THR:
                # rotate left or right depending on margin of error
                if theta_err > 0:
                    twist.angular.z = 0.5 * MAX_SPEED
                else:
                    twist.angular.z = -0.5 * MAX_SPEED
            else:
                self.state = 'FORWARD'
                twist.angular.z = 0.0

        # move TIAGo towards table
        elif self.state == 'FORWARD':
            # as long as TIAG'o is not deviating, continue moving forward
            if abs(theta_err) > ANGLE_THR:
                self.state = 'ROTATE'
                twist.linear.x = 0.0
            elif dist_diff <= LIFT_DIST and not self.isArmPosed:
                self.state = 'POSE_ARM'
            elif dist_diff < GOAL_THR:
                self.state = 'FIX_ANGLE'
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.5 * MAX_SPEED

        elif self.state == 'POSE_ARM':
            # not calling it again
            if self.current_task_idx == 0 and not self.isArmPosed:
                self.lift_arm_request()
            elif (self.current_task_idx == 1 or self.current_task_idx == 3) and not self.isArmPosed:
                self.grab_pose_request()
                pass  # DO NOT RETURN
            else:
                self.state = 'ROTATE'

        elif self.state == 'FIX_ANGLE':
            # reposition TIAGo on z-axis
            theta_err = self.target[2] - self.theta
            theta_err = math.atan2(math.sin(theta_err), math.cos(theta_err))

            if abs(theta_err) > 0.001:
                # rotate left or right depending on margin of error
                if theta_err > 0:
                    twist.angular.z = 0.3 * MAX_SPEED
                else:
                    twist.angular.z = -0.3 * MAX_SPEED

            else:
                # stop
                twist.angular.z = 0.0

                if self.current_task_idx == 0:
                    self.state = 'BACKWARD'
                    self.set_move_back_to(distance=1.0)
                elif self.current_task_idx == 1 or self.current_task_idx == 3:
                    self.state = 'PICKUP'
                elif self.current_task_idx == 2 or self.current_task_idx == 4:
                    self.state = 'PLACE_OBJ'

        elif self.state == 'PICKUP':
            # stay in PICKUP until done
            if not self.isObjectPicked:
                pass
                # self.pick_obj_request()  # sends request or checks future
            else:
                # once done:
                self.state = 'BACKWARD'
                self.set_move_back_to(distance=1.0)
        elif self.state == 'PLACE_OBJ':
            # not calling it again
            if not self.isObjectPlaced:
                self.place_obj_request()
                return

            self.state = 'BACKWARD'
            self.set_move_back_to(distance=1.0)

        elif self.state == 'BACKWARD' and self.backTargetSet:
            self.get_logger().info('Moving backward')
            dx = self.target[0] - self.position[0]
            dy = self.target[1] - self.position[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > GOAL_THR:
                if self.state == 'PLACE_OBJ':
                    twist.linear.y = -0.5 * MAX_SPEED  # move backward straight
                else:
                    twist.linear.x = -0.5 * MAX_SPEED  # move backward straight
            else:
                twist.linear.x = 0.0
                self.backTargetSet = False
                self.get_logger().info('Finished moving backward')

                self.get_logger().info(f"Ending task: {self.tasks[self.current_task_idx]['action']}")
                self.current_task_idx += 1
                self.get_logger().info(f"current_task_idx: {self.current_task_idx}")
                self.target = self.tasks[self.current_task_idx]['goal']
                
                self.isArmPosed = False # TO REVIEW
                self.isObjectPicked = False
                self.isObjectPlaced = False
                self.backTargetSet = False
                self.state = 'ROTATE' # resume navigation

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