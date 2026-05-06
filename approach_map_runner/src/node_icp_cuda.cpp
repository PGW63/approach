#include "approach_map_runner/node_icp_cuda.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

double extractRoll(const Eigen::Matrix4f& transform)
{
    return std::atan2(
        static_cast<double>(transform(2, 1)),
        static_cast<double>(transform(2, 2)));
}

double extractPitch(const Eigen::Matrix4f& transform)
{
    const double sy = std::sqrt(
        static_cast<double>(transform(2, 1) * transform(2, 1) +
                            transform(2, 2) * transform(2, 2)));
    return std::atan2(
        static_cast<double>(-transform(2, 0)),
        sy);
}

double extractYaw(const Eigen::Matrix4f& transform)
{
    return std::atan2(
        static_cast<double>(transform(1, 0)),
        static_cast<double>(transform(0, 0)));
}

Eigen::Matrix4f makePlanarTransform(double x, double y, double yaw)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    const float cos_yaw = static_cast<float>(std::cos(yaw));
    const float sin_yaw = static_cast<float>(std::sin(yaw));

    transform(0, 0) = cos_yaw;
    transform(0, 1) = -sin_yaw;
    transform(1, 0) = sin_yaw;
    transform(1, 1) = cos_yaw;
    transform(0, 3) = static_cast<float>(x);
    transform(1, 3) = static_cast<float>(y);
    return transform;
}

template<typename T>
T readOptional(const YAML::Node & node, const char * key, const T & default_value)
{
    return node && node[key] ? node[key].as<T>() : default_value;
}

approach_preprocess::PreprocessConfig loadPreprocessConfigFromYaml(
    const std::string & yaml_path,
    const approach_preprocess::PreprocessConfig & defaults)
{
    approach_preprocess::PreprocessConfig config = defaults;

    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception &) {
        return config;
    }

    const YAML::Node icp_node = root["icp"] ? root["icp"] : root;
    const YAML::Node pp = icp_node["preprocess"] ? icp_node["preprocess"] : icp_node;

    config.remove_nan_enable =
        readOptional<bool>(pp, "remove_nan_enable", config.remove_nan_enable);
    config.downsample_enable =
        readOptional<bool>(pp, "downsample_enable", config.downsample_enable);
    config.outlier_removal_enable =
        readOptional<bool>(pp, "outlier_removal_enable", config.outlier_removal_enable);
    config.robot_filter_enable =
        readOptional<bool>(pp, "robot_filter_enable", config.robot_filter_enable);
    config.ground_removal_enable =
        readOptional<bool>(pp, "ground_removal_enable", config.ground_removal_enable);
    config.voxel_leaf_size =
        readOptional<double>(pp, "voxel_leaf_size", config.voxel_leaf_size);
    config.registration_voxel_leaf_size = readOptional<double>(
        pp, "registration_voxel_leaf_size", config.registration_voxel_leaf_size);
    config.passthrough_robot_x_min =
        readOptional<double>(pp, "passthrough_robot_x_min", config.passthrough_robot_x_min);
    config.passthrough_robot_x_max =
        readOptional<double>(pp, "passthrough_robot_x_max", config.passthrough_robot_x_max);
    config.passthrough_robot_y_min =
        readOptional<double>(pp, "passthrough_robot_y_min", config.passthrough_robot_y_min);
    config.passthrough_robot_y_max =
        readOptional<double>(pp, "passthrough_robot_y_max", config.passthrough_robot_y_max);
    config.passthrough_ground_z_ =
        readOptional<double>(pp, "passthrough_ground_z", config.passthrough_ground_z_);
    config.mean_k = readOptional<int>(pp, "mean_k", config.mean_k);
    config.stddev_mul_thresh =
        readOptional<double>(pp, "stddev_mul_thresh", config.stddev_mul_thresh);

    return config;
}

}  // namespace

NodeICPCuda::NodeICPCuda() :
    Node("node_icp_cuda"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    measure_registration_metrics_ =
        this->declare_parameter<bool>("measure_registration_metrics", false);

    const auto runner_share =
        ament_index_cpp::get_package_share_directory("approach_map_runner");
    const std::string icp_config_path = runner_share + "/config/icp_config.yaml";
    preprocess_config_ = loadPreprocessConfigFromYaml(icp_config_path, preprocess_config_);
    RCLCPP_INFO(
        this->get_logger(),
        "Loaded ICP preprocess config: voxel_leaf_size=%.3f registration_voxel_leaf_size=%.3f",
        preprocess_config_.voxel_leaf_size,
        preprocess_config_.registration_voxel_leaf_size);

    accumulation_service_ = this->create_service<inha_interfaces::srv::Accumulation>(
        service_config_.accumulation_service_name,
        std::bind(&NodeICPCuda::service_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    create_publisher_topic();

    accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    latest_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    RCLCPP_INFO(this->get_logger(), "NodeICPCuda initialized");
}

// Destructor - 구현된건 없음
NodeICPCuda::~NodeICPCuda()
{
}

void NodeICPCuda::service_callback(const std::shared_ptr<inha_interfaces::srv::Accumulation::Request> request,
                          std::shared_ptr<inha_interfaces::srv::Accumulation::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received accumulation service request");
    
    if (request->start) {
        double requested_hz = hz_;
        std::string requested_target_frame;

        // 유효한 Hz 값인지 확인 : 10hz를 넘으면 라이다 데이터 센싱보다 빠름
        if (request->hz > 0.0 && request->hz <= 10.0) {
            requested_hz = request->hz;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid Hz value in service request: %f. Using default: %f", request->hz, hz_);
            response->success = false;
            return;
        }
        // 들어온 서비스 요청의 프레임이름이 map 혹은 base가 아니면 실패
        if (!approach_icp::types::checkFrameId_only_Base_and_Map(request->target_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid target frame in service request: %s. Using default: %s", request->target_frame.c_str(), frame_config_.map_frame.c_str());
            response->success = false;
            return;
        }
        requested_target_frame = request->target_frame;

        // 이미 누적 중이면 일단 멈추고 새로 시작
        if (is_accumulating) {
            stop_accumulation();
            RCLCPP_INFO(this->get_logger(), "Restarting accumulation with new parameters");
        }

        hz_ = requested_hz;
        target_frame_ = requested_target_frame;
        RCLCPP_INFO(this->get_logger(), "Starting accumulation");
        RCLCPP_INFO(this->get_logger(), "Hz: %f", hz_);
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        start_accumulation();
    } else {
        stop_accumulation();
        RCLCPP_INFO(this->get_logger(), "Stopping accumulation");
    }
    response->success = true;
}

void NodeICPCuda::create_publisher_topic()
{
    aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_config_.aligned_cloud_topic, rclcpp::SensorDataQoS());
    accumulation_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_config_.accumulation_cloud_topic, rclcpp::SensorDataQoS());
    submap_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_config_.submap_cloud_topic, rclcpp::SensorDataQoS());
}

void NodeICPCuda::create_subscription_topic()
{
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_config_.input_cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&NodeICPCuda::cloud_callback, this, std::placeholders::_1));
}

void NodeICPCuda::create_timer()
{
    if (hz_ <= 0.0f) {
        throw std::invalid_argument("timer period cannot be negative");
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz_)),
        std::bind(&NodeICPCuda::process_callback, this)
    );
}

void NodeICPCuda::start_accumulation()
{
    accumulated_cloud->clear();
    latest_cloud->clear();
    registration_frames_.clear();
    has_latest_cloud = false;
    has_last_registered_map_from_base_ = false;
    has_last_odom_from_base_ = false;
    has_last_registered_stamp_ = false;
    last_registered_map_from_base_ = Eigen::Matrix4f::Identity();
    last_odom_from_base_ = Eigen::Matrix4f::Identity();
    last_registered_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    is_accumulating = true;
    create_subscription_topic();
    create_timer();
}

void NodeICPCuda::stop_accumulation()
{
    cloud_sub_.reset();
    timer_.reset();
    hz_ = 0.0;
    target_frame_.clear();
    accumulated_cloud->clear();
    latest_cloud->clear();
    registration_frames_.clear();
    has_latest_cloud = false;
    has_last_registered_map_from_base_ = false;
    has_last_odom_from_base_ = false;
    has_last_registered_stamp_ = false;
    last_registered_map_from_base_ = Eigen::Matrix4f::Identity();
    last_odom_from_base_ = Eigen::Matrix4f::Identity();
    last_registered_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    is_accumulating = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NodeICPCuda::preprocess_cloud_for_mapping(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_base) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in_base));

    if (preprocess_config_.remove_nan_enable && !processed->empty()) {
        processed = approach_preprocess::removeNaN(processed, preprocess_config_);
    }
    if (preprocess_config_.downsample_enable && !processed->empty()) {
        processed = approach_preprocess::downsample(processed, preprocess_config_);
    }
    if (preprocess_config_.outlier_removal_enable && !processed->empty()) {
        processed = approach_preprocess::removeOutliers(processed, preprocess_config_);
    }
    if (preprocess_config_.robot_filter_enable && !processed->empty()) {
        processed = approach_preprocess::removeRobotPoints(processed, preprocess_config_);
    }

    return processed;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NodeICPCuda::preprocess_cloud_for_registration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_base) const
{
    auto processed = preprocess_cloud_for_mapping(cloud_in_base);

    // Registration uses a (typically larger) leaf size so ICP stays fast
    // even when mapping path keeps a denser cloud.
    if (preprocess_config_.downsample_enable &&
        !processed->empty() &&
        preprocess_config_.registration_voxel_leaf_size > preprocess_config_.voxel_leaf_size)
    {
        processed = approach_preprocess::downsample(
            processed, preprocess_config_.registration_voxel_leaf_size);
    }

    if (preprocess_config_.ground_removal_enable && !processed->empty()) {
        processed = approach_preprocess::removeGroundPoints(processed, preprocess_config_);
    }

    return processed;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NodeICPCuda::build_registration_submap() const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& frame : registration_frames_) {
        *submap += *frame;
    }

    submap->width = static_cast<std::uint32_t>(submap->size());
    submap->height = 1U;
    submap->is_dense = false;
    return submap;
}

void NodeICPCuda::push_registration_frame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_map)
{
    if (!cloud_in_map || cloud_in_map->empty()) {
        return;
    }

    registration_frames_.push_back(
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in_map)));

    const int max_frames = std::max(1, registration_config_.max_submap_frames);
    while (static_cast<int>(registration_frames_.size()) > max_frames) {
        registration_frames_.pop_front();
    }
}

bool NodeICPCuda::is_reasonable_registration(
    const Eigen::Matrix4f& initial_guess_map_from_base,
    const Eigen::Matrix4f& optimized_map_from_base) const
{
    const Eigen::Matrix4f relative_correction =
        initial_guess_map_from_base.inverse() * optimized_map_from_base;

    const double dx = static_cast<double>(relative_correction(0, 3));
    const double dy = static_cast<double>(relative_correction(1, 3));
    const double dz = static_cast<double>(relative_correction(2, 3));
    const double planar_translation = std::hypot(dx, dy);
    const double roll = extractRoll(relative_correction);
    const double pitch = extractPitch(relative_correction);
    const double yaw = extractYaw(relative_correction);

    const bool reasonable =
        planar_translation <= registration_config_.max_planar_translation_correction &&
        std::abs(yaw) <= registration_config_.max_yaw_correction &&
        std::abs(roll) <= registration_config_.max_roll_correction &&
        std::abs(pitch) <= registration_config_.max_pitch_correction &&
        std::abs(dz) <= registration_config_.max_z_correction;

    if (!reasonable) {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejecting registration result. dxy=%.3f dz=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
            planar_translation,
            dz,
            roll,
            pitch,
            yaw);
    }

    return reasonable;
}

Eigen::Matrix4f NodeICPCuda::project_registration_to_planar(
    const Eigen::Matrix4f& initial_guess_map_from_base,
    const Eigen::Matrix4f& optimized_map_from_base) const
{
    const Eigen::Matrix4f relative_correction =
        initial_guess_map_from_base.inverse() * optimized_map_from_base;
    const double dx = static_cast<double>(relative_correction(0, 3));
    const double dy = static_cast<double>(relative_correction(1, 3));
    const double yaw = extractYaw(relative_correction);

    return initial_guess_map_from_base * makePlanarTransform(dx, dy, yaw);
}

void NodeICPCuda::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_cloud->clear();
    pcl::fromROSMsg(*msg, *latest_cloud);
    latest_stamp = msg->header.stamp;

    has_latest_cloud = !latest_cloud->empty();
}

void NodeICPCuda::process_callback()
{
    this->get_parameter("measure_registration_metrics", measure_registration_metrics_);

    if (!has_latest_cloud || latest_cloud->empty()){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No point cloud received yet");
        return;
    }

    geometry_msgs::msg::TransformStamped base_from_sensor_transform;
    geometry_msgs::msg::TransformStamped odom_from_base_transform;
    geometry_msgs::msg::TransformStamped map_from_base_transform;
    try {
        base_from_sensor_transform = approach_icp::tf_utils::lookupTransform(
            tf_buffer_, frame_config_.base_frame, frame_config_.sensor_frame,
            latest_stamp, rclcpp::Duration::from_seconds(0.2));
        odom_from_base_transform = approach_icp::tf_utils::lookupTransform(
            tf_buffer_, frame_config_.odom_frame, frame_config_.base_frame,
            latest_stamp, rclcpp::Duration::from_seconds(0.2));
        if (!has_last_registered_map_from_base_ || !has_last_odom_from_base_) {
            map_from_base_transform = approach_icp::tf_utils::lookupTransform(
                tf_buffer_, frame_config_.map_frame, frame_config_.base_frame,
                latest_stamp, rclcpp::Duration::from_seconds(0.2));
        }
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
    }

    const auto raw_cloud_in_base = approach_icp::tf_utils::transformPointCloud(
        latest_cloud,
        approach_icp::tf_utils::transformToMatrix(base_from_sensor_transform));

    auto mapping_cloud_in_base = preprocess_cloud_for_mapping(raw_cloud_in_base);
    auto registration_cloud_in_base = preprocess_cloud_for_registration(raw_cloud_in_base);

    if (mapping_cloud_in_base->empty()) {
        RCLCPP_WARN(this->get_logger(), "Mapping cloud became empty after preprocessing");
        latest_cloud->clear();
        has_latest_cloud = false;
        return;
    }

    if (registration_cloud_in_base->empty()) {
        registration_cloud_in_base.reset(
            new pcl::PointCloud<pcl::PointXYZ>(*mapping_cloud_in_base));
    }

    const Eigen::Matrix4f current_odom_from_base =
        approach_icp::tf_utils::transformToMatrix(odom_from_base_transform);
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    if (has_last_registered_map_from_base_ && has_last_odom_from_base_) {
        const Eigen::Matrix4f relative_motion_prev_base_to_curr_base =
            last_odom_from_base_.inverse() * current_odom_from_base;
        if (has_last_registered_stamp_ && registration_config_.max_angular_velocity_for_update > 0.0) {
            const double dt_sec = std::max((latest_stamp - last_registered_stamp_).seconds(), 1e-3);
            const double yaw_delta_rad = std::abs(extractYaw(relative_motion_prev_base_to_curr_base));
            const double angular_velocity_rad_per_sec = yaw_delta_rad / dt_sec;

            if (angular_velocity_rad_per_sec > registration_config_.max_angular_velocity_for_update) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Skipping distorted frame during high rotation: %.3f rad/s",
                    angular_velocity_rad_per_sec);
                latest_cloud->clear();
                has_latest_cloud = false;
                return;
            }
        }
        initial_guess =
            last_registered_map_from_base_ * relative_motion_prev_base_to_curr_base;
    } else {
        initial_guess = approach_icp::tf_utils::transformToMatrix(map_from_base_transform);
    }
    const auto registration_target_submap = build_registration_submap();

    // 첫 스캔
    if (registration_target_submap->empty()) {
        const auto first_aligned_registration_cloud =
            approach_icp::tf_utils::transformPointCloud(registration_cloud_in_base, initial_guess);
        auto first_cloud_in_target =
            approach_icp::tf_utils::transformPointCloud(mapping_cloud_in_base, initial_guess);

        *accumulated_cloud = *first_cloud_in_target;
        if (preprocess_config_.downsample_enable) {
            accumulated_cloud = approach_preprocess::downsample(accumulated_cloud, preprocess_config_);
        }
        push_registration_frame(first_aligned_registration_cloud);
        last_registered_map_from_base_ = initial_guess;
        last_odom_from_base_ = current_odom_from_base;
        last_registered_stamp_ = latest_stamp;
        has_last_registered_map_from_base_ = true;
        has_last_odom_from_base_ = true;
        has_last_registered_stamp_ = true;

        publish_cloud(accumulated_cloud, target_frame_, publish_topic_name_.accumulation_cloud_topic);
        publish_cloud(first_cloud_in_target, target_frame_, publish_topic_name_.aligned_cloud_topic);
        publish_cloud(build_registration_submap(), target_frame_, publish_topic_name_.submap_cloud_topic);

        latest_cloud->clear();
        has_latest_cloud = false;
        return;
    }

    // 정합 수행 : VGICP CUDA로 정합 (fast_gicp 라이브러리 사용)
    const auto measurement_snapshot =
        approach_icp::performance_metrics::beginMeasurement(
            measure_registration_metrics_);
    const auto result = approach_icp::registration::registerPointClouds(
        registration_cloud_in_base,
        registration_target_submap,
        initial_guess,
        registration_config_);
    const auto measurement_result =
        approach_icp::performance_metrics::endMeasurement(measurement_snapshot);

    if (measurement_result.enabled) {
        const std::string measurement_label =
            "registration:" + registration_config_.backend;
        const std::string measurement_summary =
            approach_icp::performance_metrics::formatMeasurementSummary(
                measurement_label,
                measurement_result);
        RCLCPP_INFO(
            this->get_logger(),
            "%s src=%zu tgt=%zu",
            measurement_summary.c_str(),
            registration_cloud_in_base->size(),
            registration_target_submap->size());
    }

    if (!result.valid) {
        RCLCPP_WARN(this->get_logger(), "Registration did not converge");
        has_latest_cloud = false;
        latest_cloud->clear();
        return;
    }

    if (!is_reasonable_registration(initial_guess, result.transformation)) {
        has_latest_cloud = false;
        latest_cloud->clear();
        return;
    }

    const Eigen::Matrix4f optimized_map_from_base = result.transformation;
    const auto aligned_registration_cloud_in_map =
        approach_icp::tf_utils::transformPointCloud(
            registration_cloud_in_base,
            optimized_map_from_base);
    const auto aligned_mapping_cloud_in_map =
        approach_icp::tf_utils::transformPointCloud(
            mapping_cloud_in_base,
            optimized_map_from_base);

    publish_cloud(aligned_mapping_cloud_in_map, target_frame_, publish_topic_name_.aligned_cloud_topic);

    // 누적된 클라우드에 정합된 클라우드 추가
    push_registration_frame(aligned_registration_cloud_in_map);
    *accumulated_cloud += *aligned_mapping_cloud_in_map;
    
    if (preprocess_config_.downsample_enable) {
        accumulated_cloud = approach_preprocess::downsample(accumulated_cloud, preprocess_config_);
    }
    last_registered_map_from_base_ = optimized_map_from_base;
    last_odom_from_base_ = current_odom_from_base;
    last_registered_stamp_ = latest_stamp;
    has_last_registered_map_from_base_ = true;
    has_last_odom_from_base_ = true;
    has_last_registered_stamp_ = true;
    publish_cloud(accumulated_cloud, target_frame_, publish_topic_name_.accumulation_cloud_topic);
    publish_cloud(build_registration_submap(), target_frame_, publish_topic_name_.submap_cloud_topic);

    latest_cloud->clear();
    has_latest_cloud = false;
}

void NodeICPCuda::publish_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id,const std::string& topic_name)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_target_frame(new pcl::PointCloud<pcl::PointXYZ>());
    
    if (frame_id == frame_config_.map_frame) 
    {
        cloud_in_target_frame = cloud;
    } 
    else if (frame_id == frame_config_.base_frame) 
    {
        // base_frame으로 publish할 때는 맵->베이스 변환 적용
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = approach_icp::tf_utils::lookupTransform(
                tf_buffer_, frame_id, frame_config_.map_frame,
                latest_stamp, rclcpp::Duration::from_seconds(0.2));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        cloud_in_target_frame = approach_icp::tf_utils::transformPointCloud(cloud, approach_icp::tf_utils::transformToMatrix(transform_stamped));
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_in_target_frame, output_msg);
    output_msg.header.frame_id = frame_id;
    output_msg.header.stamp = latest_stamp;
    
    if (topic_name == publish_topic_name_.aligned_cloud_topic) {
        aligned_cloud_pub_->publish(output_msg);
    } else if (topic_name == publish_topic_name_.accumulation_cloud_topic) {
        accumulation_cloud_pub_->publish(output_msg);
    } else if (topic_name == publish_topic_name_.submap_cloud_topic) {
        submap_cloud_pub_->publish(output_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown topic name: %s", topic_name.c_str());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeICPCuda>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
