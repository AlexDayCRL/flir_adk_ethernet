#ifndef FLIR_BOSON_ETHERNET_ETHERNETCAMERA_H
#define FLIR_BOSON_ETHERNET_ETHERNETCAMERA_H

// C++ Includes
#include <string>

// Linux system includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// OpenCV Includes
#include <opencv2/opencv.hpp>

// ROS Includes
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// Spinnaker Includes
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

#include "flir_boson_ethernet/ImageEventHandler.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace flir_boson_ethernet
{

enum Encoding
{
  YUV = 0,
  RAW16 = 1
};

class EthernetCamera
{
  public:
    EthernetCamera(std::string ip, std::string camInfoPath);
    ~EthernetCamera();

    void agcBasicLinear(const cv::Mat& input_16,
                    cv::Mat* output_8,
                    const int& height,
                    const int& width);
    bool openCamera();
    bool closeCamera();

    cv::Mat getImageMatrix();

  private:
    void findMatchingCamera(CameraList camList, const unsigned int numCams);
    bool setImageAcquisition();
    void initOpenCVBuffers();
    void setCameraInfo();
    void setCameraEvents();
    void unsetCameraEvents();
    bool setImageInfo();

    std::shared_ptr<camera_info_manager::CameraInfoManager> _cameraInfo;
    int32_t _width, _height, _imageSize;
    int32_t _frame = 0;                // First frame number enumeration
    uint8_t *_bufferStart;
    CameraPtr _pCam;
    SystemPtr _system;
    ImageEventHandler *_imageHandler;

    cv::Mat _thermalImageMat;

    // Default Program options
    std::string _ipAddr, _cameraInfoPath;
    bool _zoomEnable;
};

}  // namespace flir_boson_ethernet

#endif