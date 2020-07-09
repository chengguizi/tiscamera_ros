/*
 * @Author: Cheng Huimin 
 * @Date: 2020-07-02 18:13:35 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-07-02 19:17:53
 */
#ifndef CAMERA_TRIGGER_SUBSCRIBER_HPP
#define CAMERA_TRIGGER_SUBSCRIBER_HPP

#include <ros/ros.h>
#include <mavros_msgs/CamIMUStamp.h>

#include <vector>

class CamIMUStampSubscriber{

public:
    // monotonic_time in nanosecond, and timeSyncIn in nanosecond
    typedef std::function<void(uint64_t, uint64_t)> callbackSyncOutIMU;

    CamIMUStampSubscriber(ros::NodeHandle& nh, const std::string& topic_name, size_t buffer_size = 3) :
        sub_(nh.subscribe(topic_name, buffer_size, &CamIMUStampSubscriber::callback, this))
    {

    }

    void registerCallback(callbackSyncOutIMU cb);

private:
    ros::Subscriber sub_;
    std::vector<callbackSyncOutIMU> _cb_sync_out_imu;

    void callback(const mavros_msgs::CamIMUStamp::ConstPtr);

};

void CamIMUStampSubscriber::registerCallback(CamIMUStampSubscriber::callbackSyncOutIMU cb){
  _cb_sync_out_imu.push_back(cb);
  std::cout << "Registered Callback Successfully" << std::endl;
}

void CamIMUStampSubscriber::callback(const mavros_msgs::CamIMUStamp::ConstPtr msg)
{
    // all time unit should be in nanoseconds

    uint64_t monotonic_time = msg->frame_stamp.toNSec();
    uint64_t timeSyncIn = 0;
    int seq = msg->frame_seq_id;

    std::cout << "CamIMUStampSubscriber: " << seq << ", time " << monotonic_time << std::endl;

    if (_cb_sync_out_imu.size()){
      for (auto& cb : _cb_sync_out_imu)
        cb(monotonic_time, timeSyncIn);
    }
}

#endif /* CAMERA_TRIGGER_SUBSCRIBER_HPP */
