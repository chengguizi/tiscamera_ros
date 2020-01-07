// ROS Node for Realsense D415 Streams
// Cheng Huimin, June 2018
//
//
#include "ros_publisher.hpp"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>

#define BUFFER_SIZE 20


StereoCameraPublisher::StereoCameraPublisher()
{
    StereoCameraPublisher(ros::NodeHandle("~"));
}


StereoCameraPublisher::StereoCameraPublisher(const ros::NodeHandle& nh) : _nh(nh)
{
    _it = new image_transport::ImageTransport(_nh);
    _pubLeft = new auto( _it->advertiseCamera("left/image_rect_raw",BUFFER_SIZE));
    _pubRight = new auto( _it->advertiseCamera("right/image_rect_raw",BUFFER_SIZE));
    std::cout << "Stereo Publisher initialised." << std::endl;
}

void StereoCameraPublisher::publish(cv::Mat imageLeft_cv, cv::Mat imageRight_cv, const std::string encoding, sensor_msgs::CameraInfo cameraInfo_left,
        sensor_msgs::CameraInfo cameraInfo_right,  ros::Time sensor_timestamp)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;

    // add header to the cameraInfo
    cameraInfo_left.header = header;
    cameraInfo_right.header = header;

    // std::cout << "Publishing: " << sensor_timestamp << std::endl;

    // convert to pointer format
    sensor_msgs::CameraInfoConstPtr cameraInfoPtr_left = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_left);
    sensor_msgs::CameraInfoConstPtr cameraInfoPtr_right = boost::make_shared<sensor_msgs::CameraInfo>(cameraInfo_right);

    // publish left image
    cv_bridge::CvImage imageLeft_bridge = cv_bridge::CvImage(header, \
                encoding, imageLeft_cv);

    _pubLeft->publish(imageLeft_bridge.toImageMsg(),cameraInfoPtr_left); // .toImageMsg() makes a copy of the image data

    // publish right image
    cv_bridge::CvImage imageRight_bridge = cv_bridge::CvImage(header, \
                encoding, imageRight_cv);

    _pubRight->publish(imageRight_bridge.toImageMsg(),cameraInfoPtr_right);
}

ImagePublisher::ImagePublisher()
{
    ImagePublisher(ros::NodeHandle("~"), "image");
}

ImagePublisher::ImagePublisher(const ros::NodeHandle& nh, const std::string topic) : _nh(nh)
{
    _it = new image_transport::ImageTransport(_nh);
    _pub = new auto( _it->advertiseCamera(topic, BUFFER_SIZE) );
    std::cout << topic << " Publisher initialised." << std::endl;
}

void ImagePublisher::publish(cv::Mat depth_cv, const std::string encoding, sensor_msgs::CameraInfo info, ros::Time sensor_timestamp)
{
    std_msgs::Header header;
    header.stamp = sensor_timestamp;

    sensor_msgs::CameraInfoConstPtr cameraInfoPtr = boost::make_shared<sensor_msgs::CameraInfo>(info);

    // publish left image
    cv_bridge::CvImage imageLeft_bridge = cv_bridge::CvImage(header, \
                encoding, depth_cv);

    _pub->publish(imageLeft_bridge.toImageMsg(),cameraInfoPtr); // .toImageMsg() makes a copy of the image data

}

IMUPublisher::IMUPublisher()
{
    IMUPublisher(ros::NodeHandle("~"));
}

IMUPublisher::IMUPublisher(const ros::NodeHandle& nh) : _nh(nh)
{
    _pub = _nh.advertise<sensor_msgs::Imu>("imu",BUFFER_SIZE);
    std::cout << "Imu Publisher initialised." << std::endl;
}

void IMUPublisher::publish(const float gyro[3], const float accel[3], const ros::Time timestamp)
{
    // std::cout << "IMUPublisher::publish" << std::endl;
    sensor_msgs::Imu data;

    data.header.stamp = timestamp;
    data.angular_velocity.x = gyro[0];
    data.angular_velocity.y = gyro[1];
    data.angular_velocity.z = gyro[2];

    data.linear_acceleration.x = accel[0];
    data.linear_acceleration.y = accel[1];
    data.linear_acceleration.z = accel[2];

    _pub.publish(data);
    
}

PosePublisher::PosePublisher()
{
    PosePublisher(ros::NodeHandle("~"));
}

PosePublisher::PosePublisher(const ros::NodeHandle& nh) : _nh(nh)
{
    _pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov",BUFFER_SIZE);
    std::cout << "Pose Cov Publisher initialised." << std::endl;
}

void PosePublisher::doStaticTransform(const double orintation[9])
{
    tf2::Matrix3x3 R(   orintation[0], orintation[1], orintation[2],
                        orintation[3], orintation[4], orintation[5],
                        orintation[6], orintation[7], orintation[8]);
    _tf_static = new tf2::Transform(R);
}

void PosePublisher::publish(const float position[3], const float orientation[4], const std::string frame_id, const ros::Time timestamp, const uint64_t seq)
{
    geometry_msgs::PoseWithCovarianceStamped data;

    data.header.stamp = timestamp;
    data.header.seq = seq;

    data.header.frame_id = frame_id;

    data.pose.pose.position.x = position[0];
    data.pose.pose.position.y = position[1];
    data.pose.pose.position.z = position[2];

    data.pose.pose.orientation.w = orientation[0];
    data.pose.pose.orientation.x = orientation[1];
    data.pose.pose.orientation.y = orientation[2];
    data.pose.pose.orientation.z = orientation[3];

    if (_tf_static)
    {
        tf2::Transform tf_pose;
        tf2::fromMsg(data.pose.pose, tf_pose);

        tf2::toMsg((*_tf_static).inverse() * tf_pose * (*_tf_static), data.pose.pose);
    }

    _pub.publish(data);
}