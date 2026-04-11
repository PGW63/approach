#include "approach_icp/registration.hpp"

namespace approach_icp
{
namespace registration
{

namespace
{

void configureCommonRegistrationParameters(
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>& registration,
    const RegistrationConfig& config)
{
    registration.setMaximumIterations(config.max_iterations);
    registration.setMaxCorrespondenceDistance(config.max_correspondence_distance);
    registration.setTransformationEpsilon(config.transformation_epsilon);
    registration.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
}

RegistrationResult registerPointCloudsWithIcp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    configureCommonRegistrationParameters(icp, config);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    icp.align(*aligned_cloud, initial_guess);

    RegistrationResult result;
    result.valid = icp.hasConverged();
    result.transformation = icp.getFinalTransformation();
    result.aligned_cloud = aligned_cloud;
    return result;
}

RegistrationResult registerPointCloudsWithGicp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    configureCommonRegistrationParameters(gicp, config);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    gicp.align(*aligned_cloud, initial_guess);

    RegistrationResult result;
    result.valid = gicp.hasConverged();
    result.transformation = gicp.getFinalTransformation();
    result.aligned_cloud = aligned_cloud;
    return result;
}

RegistrationResult registerPointCloudsWithFastGicp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    if (config.num_threads >= 0) {
        gicp.setNumThreads(config.num_threads);
    }
    configureCommonRegistrationParameters(gicp, config);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    gicp.align(*aligned_cloud, initial_guess);

    RegistrationResult result;
    result.valid = gicp.hasConverged();
    result.transformation = gicp.getFinalTransformation();
    result.aligned_cloud = aligned_cloud;
    return result;
}

RegistrationResult registerPointCloudsWithFastVgicp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
    vgicp.setInputSource(source_cloud);
    vgicp.setInputTarget(target_cloud);
    if (config.num_threads >= 0) {
        vgicp.setNumThreads(config.num_threads);
    }
    vgicp.setResolution(config.resolution);
    configureCommonRegistrationParameters(vgicp, config);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    vgicp.align(*aligned_cloud, initial_guess);

    RegistrationResult result;
    result.valid = vgicp.hasConverged();
    result.transformation = vgicp.getFinalTransformation();
    result.aligned_cloud = aligned_cloud;
    return result;
}

RegistrationResult registerPointCloudsWithCuda(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp;
    vgicp.setInputSource(source_cloud);
    vgicp.setInputTarget(target_cloud);
    vgicp.setResolution(config.resolution);
    configureCommonRegistrationParameters(vgicp, config);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    vgicp.align(*aligned_cloud, initial_guess);


    RegistrationResult result;
    result.valid = vgicp.hasConverged();
    result.transformation = vgicp.getFinalTransformation();
    result.aligned_cloud = aligned_cloud;
    return result;
}

}  // namespace

RegistrationResult registerPointClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    const RegistrationConfig &config)
{
    if (config.backend == "pcl_icp") {
        return registerPointCloudsWithIcp(
            source_cloud,
            target_cloud,
            initial_guess,
            config);
    }

    if (config.backend == "pcl_gicp") {
        return registerPointCloudsWithGicp(
            source_cloud,
            target_cloud,
            initial_guess,
            config);
    }

    if (config.backend == "fast_gicp") {
        return registerPointCloudsWithFastGicp(
            source_cloud,
            target_cloud,
            initial_guess,
            config);
    }

    if (config.backend == "fast_vgicp") {
        return registerPointCloudsWithFastVgicp(
            source_cloud,
            target_cloud,
            initial_guess,
            config);
    }

    return registerPointCloudsWithCuda(
        source_cloud,
        target_cloud,
        initial_guess,
        config);
}

}
}
