#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// MoveIt 2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RetractArmControl : public rclcpp::Node {
public:
    RetractArmControl(): Node("retract_arm_control"), arm_moved_(false) {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "retract_arm",
            std::bind(&RetractArmControl::handleRetractArmRequest, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "RetractArmControl node initialized as service server.");
    }

    void initializeMoveGroup(const rclcpp::Node::SharedPtr &node_shared) {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm_torso");
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }

private:
    // Service callback
    void handleRetractArmRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // no data in Trigger request

        if (!arm_moved_) {
            RCLCPP_INFO(this->get_logger(), "Received retract_arm request. Moving arm...");
            bool success = moveArmToGoal();

            response->success = success;
            response->message = success ? "Arm retracted successfully!" : "Arm failed to retracted.";
            RCLCPP_INFO(this->get_logger(), "Request sent to client.");
        } else {
            response->success = true;
            response->message = "Arm already retracted.";
        }
    }

    bool moveArmToGoal() {
        if (!move_group_) {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized. Cannot move arm.");
            return false;
        }

        std::map<std::string, double> joint_goal;
        joint_goal["torso_lift_joint"] = 0.150;
        joint_goal["arm_1_joint"] = deg2rad(29.0);
        joint_goal["arm_2_joint"] = deg2rad(-77.0);
        joint_goal["arm_3_joint"] = deg2rad(-28.0);
        joint_goal["arm_4_joint"] = deg2rad(111.0);
        joint_goal["arm_5_joint"] = deg2rad(-85.0);
        joint_goal["arm_6_joint"] = deg2rad(78.0);
        joint_goal["arm_7_joint"] = deg2rad(-0.0);
        joint_goal["gripper_left_finger_joint"] = 0.0;
        joint_goal["gripper_right_finger_joint"] = 0.0;

        move_group_->setJointValueTarget(joint_goal);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "Arm moved to the goal position.");

            arm_moved_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning failed. Arm did not move.");
        }

        return success;
    }

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    bool arm_moved_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RetractArmControl>();
    node->initializeMoveGroup(node);

    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}