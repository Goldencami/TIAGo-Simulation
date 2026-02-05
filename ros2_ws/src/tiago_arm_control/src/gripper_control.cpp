#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <gazebo_msgs/msg/model_states.hpp>
#include "std_msgs/msg/bool.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GripperControl : public rclcpp::Node {
public:
    GripperControl() : Node("gripper_control"), tf_buffer_(this->get_clock()), picked_(false), pickup_requested_(false) {
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&GripperControl::modelCallback, this, std::placeholders::_1)
        );
        
        // Subscribe to pickup trigger
        pickup_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/pickup_command", 10,
            std::bind(&GripperControl::pickupCommandCallback, this, std::placeholders::_1)
        );

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "GripperControl node created.");
    }

    void initializeMoveGroup(const rclcpp::Node::SharedPtr &node_shared) {
        arm_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm_torso");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "gripper");

        gripper_->setMaxVelocityScalingFactor(1.0);
        gripper_->setMaxAccelerationScalingFactor(1.0);

        arm_torso_->setMaxVelocityScalingFactor(0.5); // 50% of max speed
        arm_torso_->setMaxAccelerationScalingFactor(0.5); // 50% of max acceleration

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized.");
    }

private:
    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    void pickupCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Pickup command received!");
            pickup_requested_ = true;
            picked_ = false; // allow a new pickup
        }
    }

    void pickObject(const geometry_msgs::msg::PoseStamped &object_pose) {
        if (!arm_torso_ || !gripper_) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt interfaces not initialized!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting pick operation...");

        // open gripper first
        std::map<std::string, double> gripper_goal {
            {"gripper_left_finger_joint", 0.04}, // open
            {"gripper_right_finger_joint", 0.04} // open
        };
        gripper_->setJointValueTarget(gripper_goal);

        // transform object pose to MoveIt planning frame
        geometry_msgs::msg::PoseStamped target_pose;
        try {
            target_pose = tf_buffer_.transform(object_pose, "base_link", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return;
        }

        // put arm above object
        target_pose.pose.position.z += 0.45; 
        // Orientation pointing down (top-down grasp)
        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.7071;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 0.7071;

        arm_torso_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_success = (arm_torso_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (arm_success) {
            arm_torso_->execute(arm_plan);
            RCLCPP_INFO(this->get_logger(), "Arm moved above the object successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan path to target.");
        }

        moveit::planning_interface::MoveGroupInterface::Plan grippers_plan;
        bool gripper_success = (gripper_->plan(grippers_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (gripper_success) {
            gripper_->execute(grippers_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper moved above the object successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan path to target.");
        }
    }

    void modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        if (!pickup_requested_) return; // wait for command
        if (picked_) return;

        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "cocacola") {
                geometry_msgs::msg::PoseStamped object_pose;
                object_pose.header.frame_id = "odom";  // Gazebo gives odom frame
                object_pose.header.stamp = rclcpp::Time(0); // latest transform
                object_pose.pose = msg->pose[i];

                RCLCPP_INFO(this->get_logger(),
                    "Cocacola pose: (%.2f, %.2f, %.2f)",
                    object_pose.pose.position.x,
                    object_pose.pose.position.y,
                    object_pose.pose.position.z
                );

                pickObject(object_pose);

                picked_ = true; // avoid planning repeatedly
                pickup_requested_ = false;  // reset
                break;
            }
        }
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pickup_sub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_torso_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    bool picked_ = false;
    bool pickup_requested_;

    // TF2 for transforming object pose to base_link
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperControl>();
    node->initializeMoveGroup(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
