#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <gazebo_msgs/msg/model_states.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GripperControl : public rclcpp::Node {
public:
    GripperControl() : Node("gripper_control") {
        // Subscribe to Gazebo model states
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states",
            10,
            std::bind(&GripperControl::modelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "GripperControl node created.");
    }

    void initializeMoveGroup() {
        // Use 'this' pointer; no shared_from_this needed
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "arm");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "gripper");

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized.");
    }

    void modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "cocacola") {
                object_pose_.header.frame_id = "odom";
                object_pose_.header.stamp = this->now();
                object_pose_.pose = msg->pose[i];

                RCLCPP_INFO(this->get_logger(),
                    "Cocacola pose: (%.2f, %.2f, %.2f)",
                    object_pose_.pose.position.x,
                    object_pose_.pose.position.y,
                    object_pose_.pose.position.z
                );
            }
        }
    }


private:
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
    geometry_msgs::msg::PoseStamped object_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperControl>();
    node->initializeMoveGroup(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}