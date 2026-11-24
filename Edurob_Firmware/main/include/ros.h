#ifndef EDUROB_ROS_H
#define EDUROB_ROS_H

#include <string>
#include "DriveHAL.h"
using namespace EduRobV2::HAL;

void micro_ros_init(std::string IP, std::string Port, size_t DomainID, DriveHAL &DHAL);

#endif // EDUROB_ROS_H
