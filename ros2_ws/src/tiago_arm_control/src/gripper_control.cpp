#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

// MoveIt 2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GripperControl : public rclcpp::Node {
public:
    GripperControl(): Node("gripper_control"), tf_buffer_(this->get_clock()), picked_up_(false), location_fetched_(false) {
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&GripperControl::modelCallback, this, std::placeholders::_1)
        );

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "gripper",
            std::bind(&GripperControl::handleGripRequest, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "GripperControl node initialized as service server.");
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
    // Service callback
    void handleGripRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // no data in Trigger request

        if (!location_fetched_) {
            response->success = false;
            response->message = "Object location not yet detected!";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received gripper request. Moving arm...");
        bool success = pickObject();

        response->success = success;
        response->message = success ? "Gripped successfully!" : "Failed to pick object.";
    }

    void modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        if (location_fetched_) return;

        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "cocacola") {
                object_pose_.header.frame_id = "odom";  // Gazebo gives odom frame
                object_pose_.header.stamp = rclcpp::Time(0); // latest transform
                object_pose_.pose = msg->pose[i];

                RCLCPP_INFO(this->get_logger(),
                    "Cocacola pose: (%.2f, %.2f, %.2f)",
                    object_pose_.pose.position.x,
                    object_pose_.pose.position.y,
                    object_pose_.pose.position.z
                );


                location_fetched_ = true; // avoid planning repeatedly
                return;
            }
        }
    }

    bool pickObject() {
        if (!arm_torso_ || !gripper_) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt interfaces not initialized!");
            return false;
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
            target_pose = tf_buffer_.transform(object_pose_, "base_link", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return false;
        }

        // put arm above object
        target_pose.pose.position.z += 0.45; 
        // Orientation pointing down (top-down grasp)
        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.7071;
        target_pose.pose.orientation.z = 0.0;
        target_pose.pose.orientation.w = 0.7071;

        // arm_torso_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        if (arm_torso_->plan(arm_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan arm motion.");
            return false;
        }

        arm_torso_->execute(arm_plan);

        moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
        if (gripper_->plan(grip_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan gripper.");
            return false;
        }

        gripper_->execute(grip_plan);


        picked_up_ = true;
        return true;
    }

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_torso_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    geometry_msgs::msg::PoseStamped object_pose_;

    bool picked_up_;
    bool location_fetched_;
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