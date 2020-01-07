// ROS Node for Realsense D415 Streams
// Cheng Huimin, July 2018
//
// ROS Interface for publishing images

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <sensor_msgs/CameraInfo.h>

// encoding == "mono8" , "mono16"

namespace image_transport
{
    class ImageTransport;
    class CameraPublisher;
}

class StereoCameraPublisher
{
    public:
        StereoCameraPublisher();
        StereoCameraPublisher(const ros::NodeHandle& nh);

        void publish(const cv::Mat imageLeft_cv,const cv::Mat imageRight_cv, const std::string encoding, const sensor_msgs::CameraInfo cameraInfo_left, const sensor_msgs::CameraInfo cameraInfo_right, const ros::Time sensor_timestamp);
    private:
        ros::NodeHandle _nh;
        std::string _encoding;
        image_transport::ImageTransport* _it;
        image_transport::CameraPublisher* _pubLeft;
        image_transport::CameraPublisher* _pubRight;
};

class ImagePublisher
{
    public:
        ImagePublisher();
        ImagePublisher(const ros::NodeHandle& nh, const std::string topic);

        void publish(const cv::Mat depth_cv, const std::string encoding, const sensor_msgs::CameraInfo info, const ros::Time sensor_timestamp);

    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport* _it;
        image_transport::CameraPublisher* _pub;
};

class IMUPublisher
{
    public:
        IMUPublisher();
        IMUPublisher(const ros::NodeHandle& nh);

    void publish(const float gyro[3], const float accel[3], const ros::Time timestamp);

    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub;
};

namespace tf2
{   
    class Transform;
}

class PosePublisher
{
    public:
        PosePublisher();
        PosePublisher(const ros::NodeHandle& nh);

        void doStaticTransform(const double orientation[9]); // Rotation matrix

    void publish(const float position[3], const float orientation[4], const std::string frame_id, const ros::Time timestamp, const uint64_t seq);

    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub;
        tf2::Transform* _tf_static = nullptr;
};

#endif /* ROS_PUBLISHER_H */
