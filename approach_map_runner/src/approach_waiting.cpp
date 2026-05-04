#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>

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
    growth_window_sec_ = this->declare_parameter<double>("growth_window_sec", 1.5);
    growth_threshold_ = this->declare_parameter<double>("growth_threshold", 200.0);
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
  enum class Phase { Idle, Paused, Released };

  void handleSetEnable(
    const std::shared_ptr<SetEnable::Request> request,
    std::shared_ptr<SetEnable::Response> response)
  {
    if (request->enable) {
      startMonitoring();
      response->success = true;
      response->message = "approach_waiting enabled";
      RCLCPP_INFO(this->get_logger(), "Enabled: monitoring map growth inside bbox");
    } else {
      const bool keep_pending = (phase_ != Phase::Idle);
      stopMonitoring(!keep_pending);
      response->success = true;
      response->message = keep_pending ?
        "approach_waiting disabled; pending target call kept" :
        "approach_waiting disabled";
      RCLCPP_INFO(
        this->get_logger(), "Disabled%s",
        keep_pending ? "; pending target call kept" : "");
    }
  }

  void startMonitoring()
  {
    stopMonitoring(true);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&ApproachWaitingNode::mapCallback, this, std::placeholders::_1));

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(check_hz_, 0.1)));
    check_timer_ = this->create_wall_timer(
      period, std::bind(&ApproachWaitingNode::checkRobotPosition, this));

    enabled_ = true;
    phase_ = Phase::Idle;
    growth_history_.clear();
  }

  void stopMonitoring(bool cancel_pending_target_call)
  {
    map_sub_.reset();
    check_timer_.reset();
    if (cancel_pending_target_call) {
      wait_timer_.reset();
      pending_request_.reset();
      phase_ = Phase::Idle;
    }
    enabled_ = false;
    map_received_ = false;
    growth_history_.clear();
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_width_m_ = static_cast<double>(msg->info.width) * msg->info.resolution;
    map_height_m_ = static_cast<double>(msg->info.height) * msg->info.resolution;
    map_received_ = true;

    int known = 0;
    for (const auto v : msg->data) {
      if (v >= 0) ++known;
    }

    const auto stamp = this->now();
    growth_history_.emplace_back(stamp, known);

    const auto cutoff = stamp - rclcpp::Duration::from_seconds(growth_window_sec_);
    while (growth_history_.size() > 1 && growth_history_.front().first < cutoff) {
      growth_history_.pop_front();
    }
  }

  bool computeGrowthRate(double & rate_out) const
  {
    if (growth_history_.size() < 2) return false;
    const auto & first = growth_history_.front();
    const auto & last = growth_history_.back();
    const double dt = (last.first - first.first).seconds();
    if (dt < 0.2) return false;
    rate_out = static_cast<double>(last.second - first.second) / dt;
    return true;
  }

  void checkRobotPosition()
  {
    if (!enabled_ || !map_received_ || phase_ == Phase::Released) {
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

    if (!inside) return;

    double rate = 0.0;
    if (!computeGrowthRate(rate)) return;

    if (phase_ == Phase::Idle) {
      if (rate > growth_threshold_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Map growing (%.1f cells/s) at (%.3f, %.3f). Pausing.",
          rate, rx, ry);
        phase_ = Phase::Paused;
        callTargetService(true);
      }
    } else if (phase_ == Phase::Paused) {
      if (rate < growth_threshold_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Map growth settled (%.1f cells/s). Releasing.",
          rate);
        phase_ = Phase::Released;
        callTargetService(false);
      }
    }
  }

  void callTargetService(bool enable)
  {
    wait_timer_.reset();
    pending_request_ = std::make_shared<SetEnable::Request>();
    pending_request_->enable = enable;
    sendPendingRequest();
  }

  void sendPendingRequest()
  {
    if (!pending_request_) return;

    if (!target_client_->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Target service %s not available; retrying in 1.0 s",
        target_service_name_.c_str());
      wait_timer_ = this->create_wall_timer(
        1s, std::bind(&ApproachWaitingNode::sendPendingRequest, this));
      return;
    }

    auto request = pending_request_;
    pending_request_.reset();

    RCLCPP_INFO(
      this->get_logger(),
      "Calling target service %s with enable=%d",
      target_service_name_.c_str(), static_cast<int>(request->enable));

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
  double growth_window_sec_{1.5};
  double growth_threshold_{200.0};

  bool enabled_{false};
  bool map_received_{false};
  Phase phase_{Phase::Idle};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_width_m_{0.0};
  double map_height_m_{0.0};

  std::deque<std::pair<rclcpp::Time, int>> growth_history_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<SetEnable>::SharedPtr enable_service_;
  rclcpp::Client<SetEnable>::SharedPtr target_client_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  std::shared_ptr<SetEnable::Request> pending_request_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachWaitingNode>());
  rclcpp::shutdown();
  return 0;
}
