/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include <math.h>

#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "flir_adk_ethernet/CameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_adk_ethernet::CameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_adk_ethernet;

CameraController::CameraController() : BaseCameraController()
{
}

CameraController::~CameraController()
{
}

void CameraController::setupFramePublish() {
    pnh.param<float>("frame_rate", _frame_rate, 60.0);
    pnh.param<int>("ptp_time_offset_secs", _ptp_time_offset_secs, 0);
    ROS_INFO("hack flir_adk_ethernet - Got frame rate: %f.", _frame_rate);
    ROS_INFO("hack flir_adk_ethernet - Got ptp time offset: %i.", _ptp_time_offset_secs);

    capture_timer = nh.createTimer(ros::Duration(1.0 / _frame_rate),
        boost::bind(&CameraController::captureAndPublish, this, _1));
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    if (!_camera->getPTPSlaveStatus())
    {
        publishImage(ros::Time::now());
        return;
    }

    auto nsec = _camera->getActualTimestamp();
    auto sec = nsec / 1000000000u;
    nsec %= 1000000000u;

    ROS_DEBUG("flir_adk_ethernet ts %lu s %lu ns", sec, nsec);

    auto stamp = ros::Time::now();

    if (sec > std::numeric_limits<uint32_t>::max() ||
        nsec > std::numeric_limits<uint32_t>::max())
    {
        ROS_WARN("flir_adk_ethernet ts %lu s %lu ns are not in range of ros time sec/nsec - using s/w stamp."
            , sec, nsec);
    }
    else if (sec < std::abs(_ptp_time_offset_secs))
    {
        ROS_WARN("flir_adk_ethernet ts %lu s %lu ns is not large-enough for ptp offset (%d), so not applying one."
            , sec, nsec, _ptp_time_offset_secs);
    }
    else
    {
        try {
            stamp = ros::Time(sec, nsec);
            stamp += ros::Duration(_ptp_time_offset_secs);
        }
        catch (const std::runtime_error& e) {
            stamp = ros::Time::now();
            ROS_WARN("flir_adk_ethernet - WARN : %s", e.what());
        }
    }

    publishImage(stamp);
}
