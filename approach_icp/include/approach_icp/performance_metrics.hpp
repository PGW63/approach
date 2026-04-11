#pragma once

#include <chrono>
#include <ctime>
#include <string>

namespace approach_icp
{
namespace performance_metrics
{

struct MeasurementSnapshot
{
    bool enabled = false;
    std::chrono::steady_clock::time_point wall_start{};
    ::timespec cpu_start{};
    long rss_before_kb = -1;
    long hwm_before_kb = -1;
};

struct MeasurementResult
{
    bool enabled = false;
    double wall_ms = 0.0;
    double cpu_ms = 0.0;
    double cpu_to_wall_ratio = 0.0;
    long rss_before_kb = -1;
    long rss_after_kb = -1;
    long rss_delta_kb = 0;
    long hwm_before_kb = -1;
    long hwm_after_kb = -1;
    long hwm_delta_kb = 0;
};

MeasurementSnapshot beginMeasurement(bool enabled);
MeasurementResult endMeasurement(const MeasurementSnapshot& snapshot);

long readVmRssKb();
long readVmHwmKb();
std::string formatMeasurementSummary(
    const std::string& label,
    const MeasurementResult& result);

}  // namespace performance_metrics
}  // namespace approach_icp
