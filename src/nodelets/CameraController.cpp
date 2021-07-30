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
    ROS_INFO("flir_adk_ethernet - Got frame rate: %f.", _frame_rate);
    ROS_INFO("flir_adk_ethernet - Got ptp time offset: %i.", _ptp_time_offset_secs);

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

    uint64_t nsecs = fmod(_camera->getActualTimestamp(), 1e9);
    uint64_t secs = _camera->getActualTimestamp() / 1e9;
    ros::Time stamp(secs, nsecs);
    try {
        stamp += ros::Duration(_ptp_time_offset_secs);
    }
    catch (const std::runtime_error& e) {
        ROS_WARN("flir_adk_ethernet - WARN : Image Timestamp out of 32 bit range.");
        return;
    }
    publishImage(stamp);
}
