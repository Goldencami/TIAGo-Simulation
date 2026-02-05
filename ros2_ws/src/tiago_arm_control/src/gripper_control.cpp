#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// MoveIt 2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class GripperControl : public rclcpp::Node {
public:
    GripperControl(): Node("gripper_control"), picked_(false) {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "pick_obj",
            std::bind(&GripperControl::handlerPickObjRequest, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "GripperControl node initialized as service server.");
    }

    void initializeMoveGroup(const rclcpp::Node::SharedPtr &node_shared) {
        arm_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm_torso");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "gripper");

        gripper_->setMaxVelocityScalingFactor(1.0);
        gripper_->setMaxAccelerationScalingFactor(1.0);

        arm_torso_->setMaxVelocityScalingFactor(0.5); // 50% of max speed
        arm_torso_->setMaxAccelerationScalingFactor(0.5); // 50% of max acceleration

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }

private:
    // Service callback
    void handlerPickObjRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // no data in Trigger request

        if (!picked_) {
            RCLCPP_INFO(this->get_logger(), "Received pick_obj request. Moving arm...");
            bool success = pickObject();

            response->success = success;
            response->message = success ? "Picked object successfully!" : "Picked object failed.";
            RCLCPP_INFO(this->get_logger(), "Request sent to client.");
        } else {
            response->success = true;
            response->message = "Object already picked.";
        }
    }

    bool pickObject() {
        if (!arm_torso_ || !gripper_) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt interfaces not initialized!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Starting pick operation...");

        std::map<std::string, double> joint_goal;
        joint_goal["torso_lift_joint"] = 0.192;

        arm_torso_->setJointValueTarget(joint_goal);

        // open gripper first
        std::map<std::string, double> gripper_goal {
            {"gripper_left_finger_joint", 0.039}, // closing
            {"gripper_right_finger_joint", 0.039} // closing
        };
        gripper_->setJointValueTarget(gripper_goal);

        // arm_torso_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        if (arm_torso_->plan(arm_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan arm motion.");
            return false;
        }

        arm_torso_->execute(arm_plan);

        moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
        if (gripper_->plan(grip_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan grab pose.");
            return false;
        }

        gripper_->execute(grip_plan);


        picked_ = true;
        return true;
    }

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_torso_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    bool picked_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperControl>();
    node->initializeMoveGroup(node);

    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}