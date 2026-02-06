#include <memory>
#include <map>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "linkattacher_msgs/srv/attach_link.hpp"

using AttachLink = linkattacher_msgs::srv::AttachLink;
using namespace std::chrono_literals;

class GripperControl : public rclcpp::Node {
public:
  GripperControl() : Node("gripper_control"), picked_(false) {
        attach_client_ = this->create_client<AttachLink>("/ATTACHLINK");

        // Wait until service is available
        RCLCPP_INFO(this->get_logger(), "Waiting for attach service...");
        attach_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Attach service available.");

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "pick_obj",
            std::bind(&GripperControl::handlerPickObjRequest, this, 
                std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "GripperControl node initialized as service server.");
  }

private:
    // Service callback
    void handlerPickObjRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // no data in Trigger request

        if (!picked_) {
            RCLCPP_INFO(this->get_logger(), "Received pick_obj request. Moving arm...");
            bool success = pickObject("tiago", "gripper_left_finger_link", "cocacola", "link");

            response->success = success;
            response->message = success ? "Picked object successfully!" : "Picked object failed.";
            RCLCPP_INFO(this->get_logger(), "Request sent to client.");
        } else {
            response->success = true;
            response->message = "Object already picked.";
        }
    }

    bool pickObject(const std::string &model1, const std::string &link1, const std::string &model2, const std::string &link2) {
    if (!attach_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(get_logger(), "Attach service unavailable.");
        return false;
    }

    auto request = std::make_shared<AttachLink::Request>();
        request->model1_name = model1;
        request->link1_name = link1;
        request->model2_name = model2;
        request->link2_name = link2;

        // ---- NON-BLOCKING ASYNC CALL ----
        auto future = attach_client_->async_send_request(
            request,
            [this](rclcpp::Client<AttachLink>::SharedFuture future_response)
            {
                auto resp = future_response.get();

                if (resp->success) {
                    RCLCPP_INFO(this->get_logger(), "Attach succeeded: %s",
                                resp->message.c_str());
                    picked_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Attach failed: %s",
                                resp->message.c_str());
                    picked_ = false;
                }
            }
        );

        // Return immediately—DO NOT BLOCK
        RCLCPP_INFO(this->get_logger(), "Attach request sent.");
        return true;
    }


private:
    rclcpp::Client<AttachLink>::SharedPtr attach_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    bool picked_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControl>();

    // Spin to handle service requests
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}