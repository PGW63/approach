#pragma once
#include <Eigen/Core>
#include "approach_preprocess/preprocess.hpp"
#include "approach_icp/types.hpp"
#include "approach_icp/tf_utils.hpp"
#include "approach_icp/registration.hpp"
#include "approach_icp/performance_metrics.hpp"

#include "inha_interfaces/srv/accumulation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <deque>

struct PublishTopicName{
    std::string aligned_cloud_topic = "/approach/aligned_cloud";
    std::string accumulation_cloud_topic = "/approach/accumulated_cloud";
};

class NodeICPCuda : public rclcpp::Node
{
public:
    NodeICPCuda();
    ~NodeICPCuda();
private:
    void create_publisher_topic();
    void create_subscription_topic();
    void create_timer();

    void publish_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id, const std::string& topic_name);
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud_for_mapping(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_base) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud_for_registration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_base) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr build_registration_submap() const;
    void push_registration_frame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_map);
    bool is_reasonable_registration(
        const Eigen::Matrix4f& initial_guess_map_from_base,
        const Eigen::Matrix4f& optimized_map_from_base) const;
    Eigen::Matrix4f project_registration_to_planar(
        const Eigen::Matrix4f& initial_guess_map_from_base,
        const Eigen::Matrix4f& optimized_map_from_base) const;

    void stop_accumulation();
    void start_accumulation();

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void service_callback(const std::shared_ptr<inha_interfaces::srv::Accumulation::Request> request,
                          std::shared_ptr<inha_interfaces::srv::Accumulation::Response> response);
    void process_callback();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulation_cloud_pub_;

    rclcpp::Service<inha_interfaces::srv::Accumulation>::SharedPtr accumulation_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> registration_frames_;
    
    rclcpp::Time latest_stamp;
    rclcpp::Time last_registered_stamp_;
    bool has_latest_cloud = false;
    bool has_last_registered_map_from_base_ = false;
    bool has_last_odom_from_base_ = false;
    bool has_last_registered_stamp_ = false;
    Eigen::Matrix4f last_registered_map_from_base_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_odom_from_base_ = Eigen::Matrix4f::Identity();

    bool is_accumulating = false;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    float hz_ = -1.0;
    std::string target_frame_ = "map"; // map or base

    FrameConfig frame_config_;
    TopicConfig topic_config_;
    PublishTopicName publish_topic_name_;
    RegistrationConfig registration_config_;
    approach_preprocess::PreprocessConfig preprocess_config_;
    ServiceConfig service_config_;
    FRAME_ID frame_id_;
    bool measure_registration_metrics_ = false;
};
