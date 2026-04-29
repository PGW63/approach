#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "inha_interfaces/srv/set_enable.hpp"

using namespace std::chrono_literals;

class ApproachWaitingNode : public rclcpp::Node
{
public:
  using SetEnable = inha_interfaces::srv::SetEnable;

  ApproachWaitingNode()
  : Node("approach_waiting_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/approach/obstacle_map");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_nav");
    check_hz_ = this->declare_parameter<double>("check_hz", 5.0);
    wait_seconds_ = this->declare_parameter<double>("wait_seconds", 5.0);
    enable_service_name_ = this->declare_parameter<std::string>(
      "enable_service_name", "/approach/waiting/set_enable");
    // TODO: arm_id 적용
    target_service_name_ = this->declare_parameter<std::string>(
      "target_service_name", "/approach_mapping/wait");

    enable_service_ = this->create_service<SetEnable>(
      enable_service_name_,
      std::bind(
        &ApproachWaitingNode::handleSetEnable, this,
        std::placeholders::_1, std::placeholders::_2));

    target_client_ = this->create_client<SetEnable>(target_service_name_);

    RCLCPP_INFO(
      this->get_logger(),
      "approach_waiting_node started. enable_service=%s target_service=%s",
      enable_service_name_.c_str(), target_service_name_.c_str());
  }

private:
  void handleSetEnable(
    const std::shared_ptr<SetEnable::Request> request,
    std::shared_ptr<SetEnable::Response> response)
  {
    if (request->enable) {
      startMonitoring();
      response->success = true;
      response->message = "approach_waiting enabled";
      RCLCPP_INFO(this->get_logger(), "Enabled: monitoring robot position vs map bounds");
    } else {
      stopMonitoring();
      response->success = true;
      response->message = "approach_waiting disabled";
      RCLCPP_INFO(this->get_logger(), "Disabled");
    }
  }

  void startMonitoring()
  {
    stopMonitoring();

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&ApproachWaitingNode::mapCallback, this, std::placeholders::_1));

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(check_hz_, 0.1)));
    check_timer_ = this->create_wall_timer(
      period, std::bind(&ApproachWaitingNode::checkRobotPosition, this));

    enabled_ = true;
  }

  void stopMonitoring()
  {
    map_sub_.reset();
    check_timer_.reset();
    wait_timer_.reset();
    enabled_ = false;
    map_received_ = false;
    has_triggered_ = false;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_width_m_ = static_cast<double>(msg->info.width) * msg->info.resolution;
    map_height_m_ = static_cast<double>(msg->info.height) * msg->info.resolution;
    map_received_ = true;

    map_sub_.reset();

    RCLCPP_INFO(
      this->get_logger(),
      "Map bounds: origin=(%.3f, %.3f) size=(%.3f x %.3f)",
      map_origin_x_, map_origin_y_, map_width_m_, map_height_m_);
  }

  void checkRobotPosition()
  {
    if (!enabled_ || !map_received_ || has_triggered_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Could not get transform %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    const double rx = tf_msg.transform.translation.x;
    const double ry = tf_msg.transform.translation.y;

    const bool inside =
      rx >= map_origin_x_ && rx <= map_origin_x_ + map_width_m_ &&
      ry >= map_origin_y_ && ry <= map_origin_y_ + map_height_m_;

    if (inside) {
      has_triggered_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Robot entered map area at (%.3f, %.3f). Waiting %.1f s before calling %s.",
        rx, ry, wait_seconds_, target_service_name_.c_str());

      const auto wait_period = std::chrono::milliseconds(
        static_cast<int>(wait_seconds_ * 1000.0));
      wait_timer_ = this->create_wall_timer(
        wait_period, std::bind(&ApproachWaitingNode::callTargetService, this));
    }
  }

  void callTargetService()
  {
    wait_timer_.reset();

    if (!target_client_->service_is_ready()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Target service %s not available", target_service_name_.c_str());
      return;
    }

    auto request = std::make_shared<SetEnable::Request>();
    request->enable = true;

    target_client_->async_send_request(
      request,
      [this](rclcpp::Client<SetEnable>::SharedFuture future) {
        const auto resp = future.get();
        RCLCPP_INFO(
          this->get_logger(),
          "Target service response: success=%d message=%s",
          static_cast<int>(resp->success), resp->message.c_str());
      });
  }

  std::string map_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string enable_service_name_;
  std::string target_service_name_;
  double check_hz_{5.0};
  double wait_seconds_{5.0};

  bool enabled_{false};
  bool map_received_{false};
  bool has_triggered_{false};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_width_m_{0.0};
  double map_height_m_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<SetEnable>::SharedPtr enable_service_;
  rclcpp::Client<SetEnable>::SharedPtr target_client_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachWaitingNode>());
  rclcpp::shutdown();
  return 0;
}
