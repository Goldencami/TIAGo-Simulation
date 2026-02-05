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

class GrabPoseControl : public rclcpp::Node {
public:
    GrabPoseControl(): Node("grab_pose_control"), posed_(false), location_fetched_(false) {
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10,
            std::bind(&GrabPoseControl::modelCallback, this, std::placeholders::_1)
        );

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "grab_pose",
            std::bind(&GrabPoseControl::handleGrabPoseRequest, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "GrabPoseControl node initialized as service server.");
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
    void handleGrabPoseRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // no data in Trigger request

        if (!location_fetched_) {
            response->success = false;
            response->message = "Object location not yet detected!";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received gripper request. Moving arm...");
        bool success = grabPose();

        response->success = success;
        response->message = success ? "Grab posed successfully!" : "Failed to posed TIAGo.";
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

    bool grabPose() {
        if (!arm_torso_ || !gripper_) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt interfaces not initialized!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Starting posing operation...");

        std::map<std::string, double> joint_goal;
        joint_goal["torso_lift_joint"] = 0.350;
        joint_goal["arm_1_joint"] = deg2rad(4.0);
        joint_goal["arm_2_joint"] = deg2rad(15.0);
        joint_goal["arm_3_joint"] = deg2rad(-59.0);
        joint_goal["arm_4_joint"] = deg2rad(113.0);
        joint_goal["arm_5_joint"] = deg2rad(40.0);
        joint_goal["arm_6_joint"] = deg2rad(40.0);
        joint_goal["arm_7_joint"] = deg2rad(-30.0);

        arm_torso_->setJointValueTarget(joint_goal);

        // open gripper first
        std::map<std::string, double> gripper_goal {
            {"gripper_left_finger_joint", 0.044}, // open
            {"gripper_right_finger_joint", 0.044} // open
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


        posed_ = true;
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

    bool posed_;
    bool location_fetched_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GrabPoseControl>();
    node->initializeMoveGroup(node);

    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}