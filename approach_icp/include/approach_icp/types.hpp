#pragma once
#include <string>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

enum class FRAME_ID
{
    MAP,
    ODOM,
    BASE,
    LIVOX_LIDAR,
    LASER_FRAME,
    CAMERA_HEAD_LINK
};

struct FrameConfig
{
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base";
    std::string sensor_frame = "livox_lidar";
    std::string camera_frame = "camera_head_link";
};

struct TopicConfig
{
    std::string input_cloud_topic = "/livox/lidar";
    std::string aligned_cloud_topic = "/approach/aligned_cloud";
    std::string accumulation_cloud_topic = "/approach/accumulated_cloud";
};

struct ServiceConfig
{
    std::string accumulation_service_name = "accumulate";
};

struct RegistrationConfig
{
    std::string backend = "fast_gicp";
    int num_threads = 1;
    double resolution = 0.25;
    double max_correspondence_distance = 1.0;
    double transformation_epsilon = 1e-4;
    double euclidean_fitness_epsilon = 1e-3;
    int max_iterations = 100;
    int max_submap_frames = 8;
    double max_angular_velocity_for_update = 0.35;
    double max_planar_translation_correction = 1.0;
    double max_yaw_correction = 0.7;
    double max_roll_correction = 0.15;
    double max_pitch_correction = 0.15;
    double max_z_correction = 0.25;
};

struct RegistrationResult
{
    bool valid = false;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud{
        new pcl::PointCloud<pcl::PointXYZ>()
    };
};

namespace approach_icp
{
namespace types{

std::string frameIdToString(FRAME_ID frame_id);
bool checkFrameId(const std::string& frame_id, FRAME_ID expected_frame_id);
bool checkFrameId_only_Base_and_Map(const std::string& frame_id);
}}
