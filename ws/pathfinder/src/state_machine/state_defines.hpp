#pragma once

#include "../image_process/image_processer.hpp"
#include <opencv2/core/types.hpp>
#include <unordered_map>
namespace engineering_match::state_machine {

enum class CommandType {
    IDLE = 0,
    FMOVE = 1,
    BMOVE = 2,
    LMOVE = 3,
    RMOVE = 4,
    LFMOVE = 5,
    RFMOVE = 6,
    LBMOVE = 7,
    RBMOVE = 8,
    LROT = 9,
    RROT = 10,
    STOP = 11,
    FCLAMPONE = 12,
    FCLAMPTWO = 13,
    FCLAMPUP = 14,
    BCLAMP = 15,
    FPLACE = 16,
    BPLACE = 17
};

enum class DirectionType {
    FORWARD,
    BACKWARD,
};

enum class DeviceType {
    CAMERA,
    CAR
};

enum class StageType {
    STARTUP,
    GET_A_C_E,
    PLACE_A_C_E_POINT,
    GET_AND_TEMPLY_PLACE_F_POINT,
    GET_AND_TEMPLY_PLACE_G_POINT,
    COMPLETE_POINT,
    IDLE,
};

struct LocationType {
    cv::Point2f STARTUPPOINT;
    cv::Point2f CENTERTARGET;
    cv::Point2f APOINT;
    cv::Point2f BPOINT;
    cv::Point2f CPOINT;
    cv::Point2f DPOINT;
    cv::Point2f EPOINT;
    cv::Point2f FPOINT;
    cv::Point2f GPOINT;
    cv::Point2f REDTARTGET;
    cv::Point2f GREENTARGET;
    cv::Point2f BLUETARGET;
    cv::Point2f BLACKTARGET;
    cv::Point2f WHITETARGET;
};
inline LocationType Locations = {
    { 0.0f, 0.0f }, // STARTUPPOINT
    { 0.0f, 0.3375f }, // CENTERTARGET
    { -0.173f, 0.3375f }, // APOINT
    { -0.121f, 0.46f }, // BPOINT
    { 0.0f, 0.51f }, // CPOINT
    { 0.121f, 0.46f }, // DPOINT
    { 0.173f, 0.3375f }, // EPOINT
    { -0.245f, 0.087f }, // FPOINT
    { 0.245f, 0.087f }, // GPOINT
    { 0.0f, 0.683f }, // REDTARTGET
    { -0.344f, 0.3375f }, // GREENTARGET
    { 0.344f, 0.3375f }, // BLUETARGET
    { 0.245f, 0.58f }, // BLACKTARGET
    { -0.245f, 0.58f }, // WHITETARGET
};
inline std::unordered_map<image::Colors::ColorType, cv::Point2f> color_to_target_ = {
    { image::Colors::ColorType::RED, Locations.REDTARTGET },
    { image::Colors::ColorType::GREEN, Locations.GREENTARGET },
    { image::Colors::ColorType::BLUE, Locations.BLUETARGET },
    { image::Colors::ColorType::WHITE, Locations.WHITETARGET },
    { image::Colors::ColorType::BLACK, Locations.BLACKTARGET }
};
inline std::unordered_map<image::Colors::ColorType, cv::Point2f> color_to_point_ = {
    { image::Colors::ColorType::RED, Locations.APOINT },
    { image::Colors::ColorType::GREEN, Locations.BPOINT },
    { image::Colors::ColorType::BLUE, Locations.CPOINT },
    { image::Colors::ColorType::WHITE, Locations.DPOINT },
    { image::Colors::ColorType::BLACK, Locations.EPOINT }
};
}