#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "tiago_arm_control/srv/pick_object.hpp"

#include "linkattacher_msgs/srv/attach_link.hpp"
#include "linkattacher_msgs/srv/detach_link.hpp"

// MoveIt 2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using AttachLink = linkattacher_msgs::srv::AttachLink;
using DetachLink = linkattacher_msgs::srv::DetachLink;
using namespace std::chrono_literals;

class TIAGOController : public rclcpp::Node {
public:
    TIAGOController() : Node("tiago_controller"), object_detected_(false), arm_lifted_(false), 
        grab_pose_done_(false), picked_(false), attach_in_progress_(false), placed_(false), detach_in_progress_(false) {
        // subscriptions
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/model_states", 10, std::bind(&TIAGOController::modelCallback, this, std::placeholders::_1)
        );

        // services
        lift_arm_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/lift_arm", std::bind(&TIAGOController::handleLiftArmRequest, 
            this, std::placeholders::_1, std::placeholders::_2)
        );

        grab_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/grab_pose", std::bind(&TIAGOController::handleGrabPoseRequest,
            this, std::placeholders::_1, std::placeholders::_2)
        );

        pick_obj_srv_ = this->create_service<tiago_arm_control::srv::PickObject>(
            "/pick_obj", std::bind(&TIAGOController::handlePickObjRequest, 
            this, std::placeholders::_1, std::placeholders::_2)
        );

        place_obj_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/place_obj", std::bind(&TIAGOController::handlePlaceObjRequest, 
            this, std::placeholders::_1, std::placeholders::_2)
        );

        // LinkAttacher client
        attach_client_ = this->create_client<AttachLink>("/ATTACHLINK");
        detach_client_ = this->create_client<DetachLink>("/DETACHLINK");

        RCLCPP_INFO(this->get_logger(), "TIAGOController node initialized.");
    }

    void initMoveGroups() {
        auto node_ptr = shared_from_this();

        arm_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "arm_torso");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "gripper");

        arm_torso_->setMaxVelocityScalingFactor(0.5);
        arm_torso_->setMaxAccelerationScalingFactor(0.5);
        gripper_->setMaxVelocityScalingFactor(1.0);
        gripper_->setMaxAccelerationScalingFactor(1.0);
    }

private:
    // subscription callback 
    void modelCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        if (object_detected_ || object_to_pick.empty()) return;

        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == object_to_pick) {
                object_pose_.header.frame_id = "odom";  // Gazebo gives odom frame
                object_pose_.header.stamp = rclcpp::Time(0); // latest transform
                object_pose_.pose = msg->pose[i];

                RCLCPP_INFO(this->get_logger(), "%s", object_to_pick.c_str());
                RCLCPP_INFO(this->get_logger(),
                    "Object pose: (%.2f, %.2f, %.2f)",
                    object_pose_.pose.position.x,
                    object_pose_.pose.position.y,
                    object_pose_.pose.position.z
                );

                object_detected_ = true; // avoid planning repeatedly
                return;
            }
        }
    }

    // services callbacks
    void handleLiftArmRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (arm_lifted_) {
            response->success = true;
            response->message = "Arm already lifted.";
            return;
        }

        bool success = liftArm();

        response->success = success;
        response->message = success ? "Arm lifted." : "Arm lift failed.";
    }

    void handleGrabPoseRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (!object_detected_) {
            response->success = false;
            response->message = "Object not detected yet.";
            return;
        }

        if (grab_pose_done_) {
            response->success = true;
            response->message = "Already in grab pose.";
            return;
        }

        bool success = setPrePickPose();
        response->success = success;
        response->message = success ? "Grab posed successfully!" : "Failed to pose TIAGo.";
    }

    void handlePickObjRequest(
        const std::shared_ptr<tiago_arm_control::srv::PickObject::Request> request,
        std::shared_ptr<tiago_arm_control::srv::PickObject::Response> response) {
        
        object_to_pick = request->object_name; // get the desired object's name

        if (picked_) {
            response->success = true;
            response->message = "Object already picked.";
            return;
        }

        if (attach_in_progress_) {
            response->success = false;
            response->message = "Attach in progress";
            return;
        }

        attach_in_progress_ = true;
        pickObject("tiago", "gripper_left_finger_link", object_to_pick, "link");

        // let client know that it still needs to wait for a response
        response->success = false;
        response->message = "Asynchronous started.";
    }

    void handlePlaceObjRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (placed_) {
            response->success = true;
            response->message = "Object already placed.";
            return;      
        }

        if (!picked_) {
            response->success = false;
            response->message = "Nothing to detach.";
            return;
        }

        if (detach_in_progress_) {
            response->success = false;
            response->message = "Detach in progress";
            return;
        }

        detach_in_progress_ = true;
        placeObject("tiago", "gripper_left_finger_link", "cocacola", "link");

        // let client know that it still needs to wait for a response
        response->success = false;
        response->message = "Asynchronous started.";
    }

    // TIAGO's movements
    bool liftArm() {
        std::map<std::string, double> joint_goal = {
            {"torso_lift_joint", 0.350},
            {"arm_1_joint", deg2rad(4.0)},
            {"arm_2_joint", deg2rad(58.0)},
            {"arm_3_joint", deg2rad(-81.0)},
            {"arm_4_joint", deg2rad(94.0)},
            {"arm_5_joint", deg2rad(57.0)},
            {"arm_6_joint", deg2rad(-80.0)},
            {"arm_7_joint", deg2rad(0.0)}
        };

        if (!moveGroupTo(arm_torso_, joint_goal)) return false;

        arm_lifted_ = true;
        return true;
    }
    
    void pickObject(const std::string &model1, const std::string &link1, const std::string &model2, const std::string &link2) {
        if (!lowerForPick()) {
            attach_in_progress_ = false;
            return;
        }

        if (!attach_client_->wait_for_service(2s)) {
            RCLCPP_ERROR(get_logger(), "Attach service unavailable.");
            attach_in_progress_ = false;
            return;
        }

        auto request = std::make_shared<AttachLink::Request>();
        request->model1_name = model1;
        request->link1_name = link1;
        request->model2_name = model2;
        request->link2_name = link2;

        auto future = attach_client_->async_send_request(
            request,
            [this](rclcpp::Client<AttachLink>::SharedFuture future_response) {
                auto resp = future_response.get();
                attach_in_progress_ = false; 

                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(), "Attach succeeded: %s", resp->message.c_str());
                    picked_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Attach failed: %s", resp->message.c_str());
                    picked_ = false;
                }
            }
        );

        // Return immediately—DO NOT BLOCK
        RCLCPP_INFO(this->get_logger(), "Attach request sent.");
    }

    void placeObject(const std::string &model1, const std::string &link1, const std::string &model2, const std::string &link2) {
        if (!detach_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Detach service not available yet.");
            detach_in_progress_ = false;
            return;
        }

        auto request = std::make_shared<DetachLink::Request>();
        request->model1_name = model1;
        request->link1_name = link1;
        request->model2_name = model2;
        request->link2_name = link2;

        auto future = detach_client_->async_send_request(
            request,
            [this](rclcpp::Client<DetachLink>::SharedFuture future_response) {
                auto resp = future_response.get();
                detach_in_progress_ = false; 

                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(), "Detach succeeded: %s", resp->message.c_str());
                    picked_ = false;
                    placed_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Detach failed: %s", resp->message.c_str());
                    placed_ = false;
                }
            }
        );

        // logic to lift arm again
        if(!setPrePickPose()) return;
        // Return immediately—DO NOT BLOCK
        RCLCPP_INFO(this->get_logger(), "Detach request sent.");
    }

    bool setPrePickPose() {
        // This function only sets the pose ready, high above the object.
        // The robot will later reposition itself to be aligned with the object on the z-axis
        std::map<std::string, double> arm_goal = {
            {"torso_lift_joint", 0.350},
            {"arm_1_joint", deg2rad(4.0)},
            {"arm_2_joint", deg2rad(15.0)},
            {"arm_3_joint", deg2rad(-59.0)},
            {"arm_4_joint", deg2rad(113.0)},
            {"arm_5_joint", deg2rad(40.0)},
            {"arm_6_joint", deg2rad(40.0)},
            {"arm_7_joint", deg2rad(-30.0)}
        };

        std::map<std::string, double> gripper_goal = {
            {"gripper_left_finger_joint", 0.042},
            {"gripper_right_finger_joint", 0.042}
        };

        if (!moveGroupTo(arm_torso_, arm_goal)) return false;
        if (!moveGroupTo(gripper_, gripper_goal)) return false;

        grab_pose_done_ = true;
        return true;
    }

    bool lowerForPick() { // TO FIX
        std::map<std::string, double> joint_goal = {
            {"torso_lift_joint", 0.197},
            {"arm_1_joint", deg2rad(4.0)},
            {"arm_2_joint", deg2rad(15.0)},
            {"arm_3_joint", deg2rad(-59.0)},
            {"arm_4_joint", deg2rad(113.0)},
            {"arm_5_joint", deg2rad(40.0)},
            {"arm_6_joint", deg2rad(40.0)},
            {"arm_7_joint", deg2rad(-30.0)}
        };

        return moveGroupTo(arm_torso_, joint_goal);
    }

    bool moveGroupTo(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
        const std::map<std::string,double>& joints) {
        std::lock_guard<std::mutex> lock(moveit_mutex_);

        group->setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ok = group->plan(plan);
        if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Planning failed.");
            return false;
        }

        auto exec_ok = group->execute(plan);
        if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Execution failed.");
            return false;
        }

        return true;
    }


    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    std::mutex moveit_mutex_;

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Client<AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<DetachLink>::SharedPtr detach_client_;

    // services variables
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lift_arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grab_pose_srv_;
    rclcpp::Service<tiago_arm_control::srv::PickObject>::SharedPtr pick_obj_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr place_obj_srv_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_torso_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    geometry_msgs::msg::PoseStamped object_pose_;

    // movements bools
    std::string object_to_pick;
    gazebo_msgs::msg::ModelStates object_pose;
    bool object_detected_;
    bool arm_lifted_;
    bool grab_pose_done_;
    bool picked_;
    bool attach_in_progress_;
    bool placed_;
    bool detach_in_progress_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TIAGOController>();
    node->initMoveGroups();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}