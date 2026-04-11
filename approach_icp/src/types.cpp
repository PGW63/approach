#include "approach_icp/types.hpp"

namespace approach_icp
{
namespace types
{

std::string frameIdToString(FRAME_ID frame_id)
{
    switch (frame_id) {
    case FRAME_ID::MAP:
        return "map";
    case FRAME_ID::ODOM:
        return "odom";
    case FRAME_ID::BASE:
        return "base";
    case FRAME_ID::LIVOX_LIDAR:
        return "livox_lidar";
    case FRAME_ID::LASER_FRAME:
        return "laser_frame";
    case FRAME_ID::CAMERA_HEAD_LINK:
        return "camera_head_link";
    default:
        return "";
    }
}

bool checkFrameId(const std::string& frame_id, FRAME_ID expected_frame_id)
{
    return frame_id == frameIdToString(expected_frame_id);
}

bool checkFrameId_only_Base_and_Map(const std::string& frame_id)
{
    return checkFrameId(frame_id, FRAME_ID::MAP) ||
           checkFrameId(frame_id, FRAME_ID::BASE);
}

}
}
