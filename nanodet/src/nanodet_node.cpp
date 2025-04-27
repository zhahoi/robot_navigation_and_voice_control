#include "nanodet/nanodet_node.h"

NcnnNanodetRos::NcnnNanodetRos(const ros::NodeHandle& n) : m_nh(n)
{
    ///// params
    m_nh.param<int>("/nanodet_ros/target_size", target_size, 320);
    m_nh.param<float>("/nanodet_ros/prob_threshold", prob_threshold, 0.4);
    m_nh.param<float>("/nanodet_ros/nms_threshold", nms_threshold, 0.5);
    m_nh.param<bool>("/nanodet_ros/use_gpu", use_gpu, false);
    m_nh.param<std::string>("/nanodet_ros/image_topic", image_topic, "/camera/rgb/image_raw");
    m_nh.param<int>("/nanodet_ros/downsampling_infer", downsampling_infer, 1);

    ///// sub pubs
    m_img_sub = m_nh.subscribe<sensor_msgs::Image>(image_topic, 10, &NcnnNanodetRos::ImageCallback, this);
    m_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>("/camera_info", 10, &NcnnNanodetRos::CameraInfoCallback, this);

    m_detected_img_pub = m_nh.advertise<sensor_msgs::Image>("/detected_result", 10); 
    m_cam_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("/detected_result/camera_info", 10); 
    m_labels_pub = m_nh.advertise<std_msgs::String>("/tts_input", 10);
 
    // m_objects_pub = m_nh.advertise<nanodet::bboxes>("/detected_objects", 10);

    ROS_WARN("class heritated, starting node...");
}; // constructor

void NcnnNanodetRos::processImage(cv::Mat& img_in, const double& time)
{
    // Note: init yolo only once, if initialized in constructor, it will not work
    if (m_nanodet == nullptr)
    {
        m_nanodet = std::make_shared<NanoDet>(target_size, prob_threshold, nms_threshold, use_gpu);
    }
    // infer and draw
    auto start = std::chrono::high_resolution_clock::now();
    m_nanodet->load();
    
    std::vector<Object> objects;
    objects.clear();
    m_nanodet->detect(img_in, objects);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    ROS_WARN("Inference duration: %f seconds", elapsed.count());

    // handle output
    // nanodet::bboxes out_boxes_;
    // out_boxes_.header.stamp = ros::Time().fromSec(time);
    for (size_t i = 0; i < objects.size(); ++i)
    {
        // nanodet::bbox out_box_;
        auto object_ = objects[i];
        // out_box_.prob = object_.prob;
        // out_box_.label = object_.label;
        // out_box_.x = object_.rect.x;
        // out_box_.y = object_.rect.y;
        // out_box_.width = object_.rect.width;
        // out_box_.height = object_.rect.height;
        // out_boxes_.bboxes.push_back(out_box_);
        if (object_.label == 0)
        {
            ros::Time current_time = ros::Time::now();
            if ((current_time - last_tts_publish_time_).toSec() >= tts_publish_interval_) {
                std_msgs::String msg;
                msg.data = "检测到行人";
                m_labels_pub.publish(msg);
                last_tts_publish_time_ = current_time;  // 更新时间戳
            }
        }
    }

    // publish
    // if (out_boxes_.bboxes.size() > 0)
    // {
    //     m_objects_pub.publish(out_boxes_);
    // }

    if (objects.size() > 0) {
        ROS_INFO("Detected %zu objects", objects.size());
    } else {
        ROS_WARN("No objects detected.");
    }

    // draw fps and detect result
    if (objects.size() > 0) {
        m_nanodet->draw(img_in, objects);
    }
    else {
        m_nanodet->draw_unsupported(img_in);
    }

    m_nanodet->draw_fps(img_in);

    // publish image
    std_msgs::Header image_header;
    image_header.stamp = ros::Time().fromSec(time);
    image_header.frame_id = "kinect_v2";
    cv_bridge::CvImage bridge_img_ = cv_bridge::CvImage(image_header, sensor_msgs::image_encodings::BGR8, img_in);
    // cv_bridge::CvImage bridge_img_ = cv_bridge::CvImage(out_boxes_.header, sensor_msgs::image_encodings::BGR8, img_in);
    
    sensor_msgs::Image _raw_img_msg;
    bridge_img_.toImageMsg(_raw_img_msg);
    m_detected_img_pub.publish(_raw_img_msg);

    m_last_cam_info.header = image_header; 
    m_cam_info_pub.publish(m_last_cam_info); 

    return;
}

void NcnnNanodetRos::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    m_last_cam_info = *msg;
}

void NcnnNanodetRos::ImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Received image message");
    m_counter++;
    if (m_counter % downsampling_infer == 0)
    {
        cv::Mat img_in_ = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
        processImage(img_in_, msg->header.stamp.toSec());
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nanodet");
    ros::NodeHandle n("~");

    NcnnNanodetRos ncnn_nanodet_ros(n);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}