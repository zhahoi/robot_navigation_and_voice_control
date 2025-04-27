#ifndef NANODET_NODE_H
#define NANODET_NODE_H

/// C++ standard headers
#include <string>
#include <vector>
#include <chrono>
#include <ctime>
/// Main NCNN NANODET header
#include "nanodet/nanodet.h"
/// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
/// this package headers
#include "nanodet/bbox.h"
#include "nanodet/bboxes.h"

class NcnnNanodetRos
{
private:
    ///// nanodet
    int target_size;
    float prob_threshold;
    float nms_threshold;
    bool use_gpu;
    std::string ncnn_param_path;
    std::string ncnn_bin_path;
    std::string image_topic;
    int downsampling_infer;

    std::shared_ptr<NanoDet> m_nanodet = nullptr;
    int m_counter = 0;

    ///// ros and tf
    ros::NodeHandle m_nh;
    ros::Subscriber m_img_sub, m_cam_info_sub;
    ros::Publisher m_detected_img_pub, m_labels_pub, m_cam_info_pub;
    double tts_publish_interval_ = 1.25;
    ros::Time last_tts_publish_time_;

    sensor_msgs::CameraInfo m_last_cam_info;

    ///// Functions
    void ImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void processImage(cv::Mat& img_in, const double& time);
    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

public:
    NcnnNanodetRos(const ros::NodeHandle& n); // constructor
    ~NcnnNanodetRos(){}; // destructor
};

#endif