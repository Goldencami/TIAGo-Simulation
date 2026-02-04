#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <gazebo_msgs/msg/model_states.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GripperControl : public rclcpp::Node {
public:
    GripperControl() : Node("gripper_control") {
        // Subscribe to Gazebo model states
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&GripperControl::modelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "GripperControl node created.");
    }

    void initializeMoveGroup(const rclcpp::Node::SharedPtr &node_shared) {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "gripper");

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized.");
    }

private:
    void pickObject(const geometry_msgs::msg::PoseStamped &object_pose) {
        if (!arm_ || !gripper_) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt interfaces not initialized!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Starting pick operation...");

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";  // MoveIt fixed frame
        target_pose.header.stamp = this->now();

        target_pose.pose.position.x = object_pose.pose.position.x;
        target_pose.pose.position.y = object_pose.pose.position.y;
        target_pose.pose.position.z = object_pose.pose.position.z + 0.20;

        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.7071;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 0.7071;

        arm_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            arm_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Arm moved successfully.");
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan path to target.");
        }

    }

    void modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        // only pick once
        if (picked_)
            return;

        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "cocacola") {
                geometry_msgs::msg::PoseStamped object_pose;
                object_pose.header.frame_id = "base_link";
                object_pose.header.stamp = this->now();
                object_pose.pose = msg->pose[i];

                RCLCPP_INFO(this->get_logger(),
                    "Cocacola pose: (%.2f, %.2f, %.2f)",
                    object_pose.pose.position.x,
                    object_pose.pose.position.y,
                    object_pose.pose.position.z
                );

                pickObject(object_pose);
                picked_ = true; // avoid planning repeatedly
            }
        }
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
    bool picked_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperControl>();
    node->initializeMoveGroup(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
