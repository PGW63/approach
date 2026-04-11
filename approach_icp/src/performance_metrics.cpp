#include "approach_icp/performance_metrics.hpp"

#include <fstream>
#include <sstream>
#include <string>

namespace approach_icp
{
namespace performance_metrics
{

namespace
{

long readFieldKb(const std::string& key)
{
    std::ifstream status_stream("/proc/self/status");
    if (!status_stream.is_open()) {
        return -1;
    }

    std::string line;
    const std::string prefix = key + ":";
    while (std::getline(status_stream, line)) {
        if (line.rfind(prefix, 0) != 0) {
            continue;
        }

        std::istringstream line_stream(line.substr(prefix.size()));
        long value = -1;
        line_stream >> value;
        if (!line_stream.fail()) {
            return value;
        }
    }

    return -1;
}

double timespecToMilliseconds(const ::timespec& timespec_value)
{
    return static_cast<double>(timespec_value.tv_sec) * 1000.0 +
           static_cast<double>(timespec_value.tv_nsec) / 1e6;
}

}  // namespace

MeasurementSnapshot beginMeasurement(bool enabled)
{
    MeasurementSnapshot snapshot;
    snapshot.enabled = enabled;

    if (!enabled) {
        return snapshot;
    }

    snapshot.wall_start = std::chrono::steady_clock::now();
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &snapshot.cpu_start);
    snapshot.rss_before_kb = readVmRssKb();
    snapshot.hwm_before_kb = readVmHwmKb();
    return snapshot;
}

MeasurementResult endMeasurement(const MeasurementSnapshot& snapshot)
{
    MeasurementResult result;
    result.enabled = snapshot.enabled;

    if (!snapshot.enabled) {
        return result;
    }

    const auto wall_end = std::chrono::steady_clock::now();
    ::timespec cpu_end{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_end);

    result.wall_ms = std::chrono::duration<double, std::milli>(
        wall_end - snapshot.wall_start).count();
    result.cpu_ms = timespecToMilliseconds(cpu_end) -
                    timespecToMilliseconds(snapshot.cpu_start);
    result.cpu_to_wall_ratio =
        result.wall_ms > 0.0 ? result.cpu_ms / result.wall_ms : 0.0;

    result.rss_before_kb = snapshot.rss_before_kb;
    result.rss_after_kb = readVmRssKb();
    result.hwm_before_kb = snapshot.hwm_before_kb;
    result.hwm_after_kb = readVmHwmKb();

    if (result.rss_before_kb >= 0 && result.rss_after_kb >= 0) {
        result.rss_delta_kb = result.rss_after_kb - result.rss_before_kb;
    }

    if (result.hwm_before_kb >= 0 && result.hwm_after_kb >= 0) {
        result.hwm_delta_kb = result.hwm_after_kb - result.hwm_before_kb;
    }

    return result;
}

long readVmRssKb()
{
    return readFieldKb("VmRSS");
}

long readVmHwmKb()
{
    return readFieldKb("VmHWM");
}

std::string formatMeasurementSummary(
    const std::string& label,
    const MeasurementResult& result)
{
    std::ostringstream stream;
    stream << "[" << label << "] "
           << "wall=" << result.wall_ms << " ms "
           << "cpu=" << result.cpu_ms << " ms "
           << "cpu/wall=" << result.cpu_to_wall_ratio;

    if (result.rss_after_kb >= 0) {
        stream << " rss=" << result.rss_after_kb
               << " KB (delta=" << result.rss_delta_kb << " KB)";
    }

    if (result.hwm_after_kb >= 0) {
        stream << " hwm=" << result.hwm_after_kb
               << " KB (delta=" << result.hwm_delta_kb << " KB)";
    }

    return stream.str();
}

}  // namespace performance_metrics
}  // namespace approach_icp
