#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// MoveIt 2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveArmControl : public rclcpp::Node {
    public:
        MoveArmControl(): Node("move_arm_control"), arm_moved_(false) {
            // Subscriber to /move_arm_command
            arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/move_arm_command", 10,
                std::bind(&MoveArmControl::moveArmCallback, this, std::placeholders::_1));

            // Publisher to /arm_movement_status
            arm_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm_movement_status", 10);

            RCLCPP_INFO(this->get_logger(), "MoveArmControl node initialized.");
        }

        // initialize MoveGroupInterface
        void initializeMoveGroup(const rclcpp::Node::SharedPtr &node_shared) {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm_torso");

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
        }

    private:
        void moveArmToGoal() {
            if (!move_group_) {
                RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized. Cannot move arm.");
                return;
            }

            // joint goal positions
            std::map<std::string, double> joint_goal;
            joint_goal["torso_lift_joint"] = 0.340;
            joint_goal["arm_1_joint"] = deg2rad(36.0);
            joint_goal["arm_2_joint"] = deg2rad(-2.0);
            joint_goal["arm_3_joint"] = deg2rad(-95.0);
            joint_goal["arm_4_joint"] = deg2rad(131.0);
            joint_goal["arm_5_joint"] = deg2rad(17.0);
            joint_goal["arm_6_joint"] = deg2rad(-80.0);
            joint_goal["arm_7_joint"] = deg2rad(0.0);

            move_group_->setJointValueTarget(joint_goal);

            move_group_->setMaxVelocityScalingFactor(0.5); // 50% of max speed
            move_group_->setMaxAccelerationScalingFactor(0.5); // 50% of max acceleration

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                move_group_->execute(my_plan);
                RCLCPP_INFO(this->get_logger(), "Arm moved to the goal position.");

                // publish arm movement is done
                auto done_msg = std_msgs::msg::Bool();
                done_msg.data = true;
                arm_done_pub_->publish(done_msg);
                RCLCPP_INFO(this->get_logger(), "Published arm movement done.");
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Planning failed. Arm did not move.");
            }
        }

        void moveArmCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !arm_moved_) {
                RCLCPP_INFO(this->get_logger(), "Received arm move command. Moving arm...");
                moveArmToGoal();
                arm_moved_ = true;
            }
        }

        double deg2rad(double deg) {
            return deg * M_PI / 180.0;
        }

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_done_pub_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        bool arm_moved_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveArmControl>();
    node->initializeMoveGroup(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}