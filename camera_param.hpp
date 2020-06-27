#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

class CameraParam{

public:

    // device param
    std::string camera_ns;
    std::string type;
    std::string camera_sn;
    std::string hardware_sync_mode = "none";

    // type param
    int width = 0,height = 0, gst_max_frame_rate = 0;
    std::string exposure_mode = "auto";
    std::string format = "GRAY16_LE";

    sensor_msgs::CameraInfo camera_info;

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

    static std::vector<std::shared_ptr<CameraParam>> list;
    
    static void loadCameras(const ros::NodeHandle& nh);
    void loadParam(const std::string& topic_ns);
    void loadCalibration(const ros::NodeHandle &nh);
    void loadExposureControlParam(const std::string& type);
};

void CameraParam::loadCameras(const ros::NodeHandle& nh)
{
    std::cout  << "CameraParam: " << "loading cameras at namespace " << nh.getNamespace() << std::endl;

    std::vector<std::string> camera_ns_vec;
    nh.getParam("devices", camera_ns_vec);

    list.resize(camera_ns_vec.size());

    for(size_t i = 0; i < list.size(); i++){
        list[i].reset(new CameraParam);
        list[i]->camera_ns = camera_ns_vec[i];
        std::cout << "device: " << nh.getNamespace() << "/" <<list[i]->camera_ns << std::endl;
    }

    std::cout << "Loaded " << list.size() << " camera namespaces." << std::endl;

}

void CameraParam::loadParam(const std::string& camera_ns)
{
    std::cout << "CameraParam: " << "Loading Param for " << camera_ns << std::endl;
    this->camera_ns = camera_ns;
    ros::NodeHandle nh_local("~/" + camera_ns);

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

void CameraParam::loadCalibration(const ros::NodeHandle &nh)
{
    std::cout  << "CameraParam: " << "loading calibration" << std::endl;

    // uint32 height
    // uint32 width
    // string distortion_model
    // float64[9]  K # 6  
    // float64[12] P 


    // Currently only support double sphere camera model
    // nh.param<std::string>(camera_ns + "/distortion_model",
    //   camera_info.distortion_model, std::string("ds"));

    std::vector<int> resolution_temp(2);
    nh.getParam(camera_ns + "/resolution", resolution_temp);
    camera_info.width = resolution_temp[0];
    camera_info.height = resolution_temp[1];


    std::vector<double> intrinsics_temp(6);
    nh.getParam(camera_ns + "/intrinsics", intrinsics_temp);
    camera_info.K[0] = intrinsics_temp[0];
    camera_info.K[1] = intrinsics_temp[1];
    camera_info.K[2] = intrinsics_temp[2];
    camera_info.K[3] = intrinsics_temp[3];
    camera_info.K[4] = intrinsics_temp[4];
    camera_info.K[5] = intrinsics_temp[5];

    cv::Mat T_cam0_cam1 = utils::getTransformCV(nh, camera_ns + "/T_cn_cnm1");
    camera_info.P[0] = T_cam0_cam1.at<double>(0, 0);
    camera_info.P[1] = T_cam0_cam1.at<double>(0, 1);
    camera_info.P[2] = T_cam0_cam1.at<double>(0, 2);
    camera_info.P[3] = T_cam0_cam1.at<double>(0, 3);
    camera_info.P[4] = T_cam0_cam1.at<double>(1, 0);
    camera_info.P[5] = T_cam0_cam1.at<double>(1, 1);
    camera_info.P[6] = T_cam0_cam1.at<double>(1, 2);
    camera_info.P[7] = T_cam0_cam1.at<double>(1, 3);
    camera_info.P[8] = T_cam0_cam1.at<double>(2, 0);
    camera_info.P[9] = T_cam0_cam1.at<double>(2, 1);
    camera_info.P[10] = T_cam0_cam1.at<double>(2, 2);
    camera_info.P[11] = T_cam0_cam1.at<double>(2, 3);

    ROS_INFO_STREAM(camera_info);
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