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

    std::string tonemapping_mode = "none";
    float tonemapping_intensity = 1.0;
    float tonemapping_global_brightness = 0.5;

    // Exposure control param
    int initial_exposure, initial_gain;
    float initial_gamma;

    int exposure_auto_reference = 128;

    bool highlight_reduction = false;

    bool auto_upper_limit_exposure = false;

    int lower_limit_exposure = 333, upper_limit_exposure = 10000; // us
    int lower_limit_gain = 0 , upper_limit_gain = 200;

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

    nh_local.getParam("tonemapping_mode", tonemapping_mode);
    nh_local.getParam("tonemapping_intensity", tonemapping_intensity);
    nh_local.getParam("tonemapping_global_brightness", tonemapping_global_brightness);

    nh_local.getParam("highlight_reduction", highlight_reduction);

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
    nh_local.getParam("initial_gamma", initial_gamma);

    nh_local.getParam("exposure_auto_reference", exposure_auto_reference);

    nh_local.getParam("lower_limit_exposure", lower_limit_exposure);
    nh_local.getParam("upper_limit_exposure", upper_limit_exposure);

    nh_local.getParam("lower_limit_gain", lower_limit_gain);
    nh_local.getParam("upper_limit_gain", upper_limit_gain);

}