#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

# Parameters
MAX_SPEED = 0.7
DIST_THR = 0.10
ANGLE_THR = 0.05
STOP_DIST = 0.70     # For arm tasks

class TiagoBaseControl(Node):
    def __init__(self):
        super().__init__('tiago_base_control')

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth_odom', self.curr_position_callback, 10)

        self.arm_status_sub = self.create_subscription(
            Bool, '/arm_movement_status', self.arm_status_callback, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        self.arm_move_pub = self.create_publisher(Bool, '/move_arm_command', 10)
        self.pickup_cmd_pub = self.create_publisher(Bool, '/pickup_command', 10)

        # --- Internal state ---
        self.position = (0.0, 0.0)
        self.theta = 0.0

        self.state = "NAV_MOVE"   # NAV_MOVE, NAV_FINAL_ROTATE, ARM_EXEC, BACKWARD
        self.arm_cmd_sent = False
        self.arm_done = False

        self.tasks = [
            {'goal': (-0.600035, 0.013528, 0), 'action': 'move_arm_above_table'},
            {'goal': (5.451085, -2.887168, 0), 'action': 'pickup'},
            {'goal': (1.0, 1.0, 0), 'action': 'place_can'},
            {'goal': (5.451085, -3.305861, 0), 'action': 'pickup'},
            {'goal': (1.0, 1.0, 0), 'action': 'place_cup'}, 
        ]
        self.current_task_idx = 0
        self.target = self.tasks[0]['goal']

        self.back_target = None
        self.timer = self.create_timer(0.1, self.loop)

    # -------------------------------
    # Callbacks
    # -------------------------------
    def curr_position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        r = msg.pose.pose.orientation
        self.theta = math.atan2(
            2 * (r.w * r.z + r.x * r.y),
            1 - 2 * (r.y**2 + r.z**2)
        )

    def arm_status_callback(self, msg):
        if msg.data:
            self.arm_done = True
            self.get_logger().info("Arm movement completed.")

    # -------------------------------
    # Helper functions
    # -------------------------------
    def target_distance(self):
        x, y = self.position
        tx, ty = self.target[:2]
        return math.hypot(tx - x, ty - y)

    def heading_error(self):
        x, y = self.position
        tx, ty = self.target[:2]
        desired = math.atan2(ty - y, tx - x)
        err = desired - self.theta
        return math.atan2(math.sin(err), math.cos(err))

    # -------------------------------
    # Arm Control
    # -------------------------------
    def send_arm_action(self, action):
        msg = Bool()
        msg.data = True

        if action == "move_arm_above_table":
            self.arm_move_pub.publish(msg)
        elif action == "pickup":
            self.pickup_cmd_pub.publish(msg)
        else:
            self.get_logger().warning(f"Unsupported arm action: {action}")

        self.get_logger().info(f"Executing arm action: {action}")
        self.arm_cmd_sent = True
        self.arm_done = False

    # -------------------------------
    # Main Loop
    # -------------------------------
    def loop(self):
        if self.current_task_idx >= len(self.tasks):
            self.get_logger().info("All tasks complete.")
            self.timer.cancel()
            return

        action = self.tasks[self.current_task_idx]['action']
        dist = self.target_distance()
        heading_err = self.heading_error()

        twist = Twist()

        # ----------------------------------------------------
        # PHASE 1: ARM EXECUTION
        # ----------------------------------------------------
        if self.state == "ARM_EXEC":
            if not self.arm_done:
                # Hold still until arm completes
                self.cmd_pub.publish(twist)
                return

            # Arm is done → go backward then next task
            self.start_backward()
            return

        # ----------------------------------------------------
        # PHASE 2: BACKWARD
        # ----------------------------------------------------
        if self.state == "BACKWARD":
            if self.back_target is None:
                self.state = "NAV_MOVE"
                return

            bx, by = self.back_target
            x, y = self.position
            if math.hypot(bx - x, by - y) > DIST_THR:
                twist.linear.x = -0.5 * MAX_SPEED
            else:
                self.back_target = None
                self.state = "NAV_MOVE"
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                return

            self.cmd_pub.publish(twist)
            return

        # ----------------------------------------------------
        # PHASE 3: ARM TRIGGER ZONE
        # ----------------------------------------------------
        if action in ["move_arm_above_table", "pickup"]:
            if dist < STOP_DIST and not self.arm_cmd_sent:
                # Stop & execute arm action
                self.cmd_pub.publish(twist)
                self.send_arm_action(action)
                self.state = "ARM_EXEC"
                return

        # ----------------------------------------------------
        # PHASE 4: NAVIGATION — MOVE
        # ----------------------------------------------------
        if self.state == "NAV_MOVE":
            if dist < DIST_THR:
                self.state = "NAV_FINAL_ROTATE"
                self.cmd_pub.publish(twist)
                return

            if abs(heading_err) > ANGLE_THR:
                twist.angular.z = 0.5 * MAX_SPEED
            else:
                twist.linear.x = 0.5 * MAX_SPEED

            self.cmd_pub.publish(twist)
            return

        # ----------------------------------------------------
        # PHASE 5: NAVIGATION — FINAL ORIENTATION
        # ----------------------------------------------------
        if self.state == "NAV_FINAL_ROTATE":
            yaw_target = self.target[2]
            yaw_err = yaw_target - self.theta
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            if abs(yaw_err) > ANGLE_THR:
                twist.angular.z = 0.3 * MAX_SPEED
                self.cmd_pub.publish(twist)
                return

            # Finished this task
            self.get_logger().info(f"Task {self.current_task_idx} complete.")
            self.advance_task()
            return

        # Default publish
        self.cmd_pub.publish(twist)

    # -------------------------------
    # Task & Motion Helpers
    # -------------------------------
    def start_backward(self, distance=1.0):
        x, y = self.position
        self.back_target = (
            x - distance * math.cos(self.theta),
            y - distance * math.sin(self.theta)
        )
        self.state = "BACKWARD"
        self.get_logger().info("Backing up before next task.")

    def advance_task(self):
        self.current_task_idx += 1
        if self.current_task_idx < len(self.tasks):
            self.target = self.tasks[self.current_task_idx]['goal']
            self.arm_cmd_sent = False
            self.arm_done = False
            self.start_backward()
        else:
            self.get_logger().info("All tasks complete.")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TiagoBaseControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()