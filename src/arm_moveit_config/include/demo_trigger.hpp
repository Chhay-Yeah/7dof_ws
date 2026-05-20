// Helper to add re-trigger capability to any demo node.
// Provides:
//   - subscription on /demo/run (std_msgs/Empty)
//   - service /demo/run_service (std_srvs/Trigger)
// On either, sets `request` true. Caller drains it with consume() and re-runs.

#pragma once
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

class DemoTrigger {
public:
    explicit DemoTrigger(rclcpp::Node* node) {
        sub_ = node->create_subscription<std_msgs::msg::Empty>(
            "/demo/run", 10,
            [this, node](std_msgs::msg::Empty::SharedPtr) {
                RCLCPP_INFO(node->get_logger(), ">>> /demo/run topic received");
                request_ = true;
            });
        srv_ = node->create_service<std_srvs::srv::Trigger>(
            "/demo/run_service",
            [this, node](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                RCLCPP_INFO(node->get_logger(), ">>> /demo/run_service called");
                request_ = true;
                res->success = true;
                res->message = "Demo queued";
            });
    }

    bool consume() { return request_.exchange(false); }

private:
    std::atomic<bool> request_{false};
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};
