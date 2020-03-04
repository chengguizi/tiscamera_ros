#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

class CameraParam{

public:

    // device param
    std::string topic_ns;
    std::string type;
    std::string camera_sn;
    std::string hardware_sync_mode = "none";

    // type param
    int width = 0,height = 0, gst_max_frame_rate = 0;
    std::string exposure_mode = "auto";
    std::string format = "GRAY16_LE";

    // Exposure control param
    int initial_exposure, initial_gain;

    static std::vector<std::string> camera_list;
    static std::vector<std::string>  loadCameras(const ros::NodeHandle& nh);
    void loadParam(const std::string& topic_ns);
    void loadExposureControlParam(const std::string& type);
};

std::vector<std::string>  CameraParam::loadCameras(const ros::NodeHandle& nh)
{
    // // std::cout << nh_local.getNamespace() << std::endl;
    nh.getParam("devices", camera_list);

    return camera_list;
}

void CameraParam::loadParam(const std::string& topic_ns)
{
    std::cout << "CameraParam: " << "Loading Param for " << topic_ns << std::endl;
    this->topic_ns = topic_ns;
    ros::NodeHandle nh_local("~/" + topic_ns);

    // Obtain Device Specific Params
    nh_local.getParam("type",type);
    nh_local.getParam("sn",camera_sn);

    nh_local.getParam("hardware_sync_mode", hardware_sync_mode);
    nh_local.getParam("exposure_mode", exposure_mode); // internal, or custom

    // Obtain Type Specific Params
    ros::NodeHandle nh_type("~/" + type);
    nh_type.getParam("width",width);
    nh_type.getParam("height",height);
    nh_type.getParam("gst_max_frame_rate",gst_max_frame_rate);
    nh_type.getParam("format",format);
}

void CameraParam::loadExposureControlParam(const std::string& type)
{
    std::cout << "CameraParam: " << "Loading Exposure Control for " << type << std::endl;
    ros::NodeHandle nh_local("~/" + type);
    nh_local.getParam("initial_exposure",initial_exposure);
    nh_local.getParam("initial_gain",initial_gain);

}