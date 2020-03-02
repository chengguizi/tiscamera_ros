/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-17 11:39:39 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2019-09-25 16:33:49
 */

#include <tiscamera_interface/tiscamera_interface.hpp>
#include <gst/gst.h>

#include <unistd.h>

#include <thread>
#include <vector>

#include <mutex>

#include <ros/ros.h>
#include <imu_vn_100/imu_vn_100.h>

#include <sensor_msgs/image_encodings.h>

#include "camera_imu_sync.hpp"
#include "ros_publisher.hpp"
#include "utils.hpp"

#include "camera_param.hpp"

using namespace std;
typedef CameraIMUSync<VnDeviceCompositeData, TisCameraManager::FrameData, 2> CameraIMUSyncN;
unsigned char MASK = 0;


StereoCameraPublisher* _pub;
IMUPublisher* _pub_imu;
sensor_msgs::CameraInfo left_info, right_info;

std::vector<std::string> CameraParam::camera_list;

// template <class TimuData, class TcameraData, int Ncamera>
void callbackSynced_handler(CameraIMUSyncN::MetaFrame frame)
{
    // convert data to cv::Mat

    auto& left_frame = frame.camera[0];
    auto& right_frame = frame.camera[1];

    

    auto left = cv::Mat(cv::Size(left_frame.width, left_frame.height), CV_16UC1, left_frame.image_data, cv::Mat::AUTO_STEP);
    auto right = cv::Mat(cv::Size(right_frame.width, right_frame.height), CV_16UC1, right_frame.image_data, cv::Mat::AUTO_STEP);
    

    // (const cv::Mat imageLeft_cv,const cv::Mat imageRight_cv, const std::string encoding, const sensor_msgs::CameraInfo cameraInfo_left, const sensor_msgs::CameraInfo cameraInfo_right, const ros::Time sensor_timestamp);
    _pub->publish(left, right, "mono16", left_info, right_info, ros::Time().fromNSec(frame.trigger_time));
    // .camera_sn
    return;
}



// void loadParam(const std::string& topic_ns)
// {
//     ros::NodeHandle nh_local("~/" + topic_ns);
//     nh_local.getParam("initial_exposure", initial_exposure);
//     nh_local.getParam("initial_gain", initial_gain);
//     std::cout << "initial_exposure = " << initial_exposure << std::endl;
//     std::cout << "initial_gain = " << initial_gain << std::endl;
// }

void loadCalibration(const ros::NodeHandle &nh)
{
    // uint32 height
    // uint32 width
    // string distortion_model
    // float64[9]  K # 6  
    // float64[12] P 
    nh.param<string>("cam0/distortion_model",
      left_info.distortion_model, string("ds"));
    nh.param<string>("cam1/distortion_model",
      right_info.distortion_model, string("ds"));

    vector<int> cam0_resolution_temp(2);
    nh.getParam("cam0/resolution", cam0_resolution_temp);
    left_info.width = cam0_resolution_temp[0];
    left_info.height = cam0_resolution_temp[1];

    vector<int> cam1_resolution_temp(2);
    nh.getParam("cam1/resolution", cam1_resolution_temp);
    right_info.width = cam1_resolution_temp[0];
    right_info.height = cam1_resolution_temp[1];

    vector<double> cam0_intrinsics_temp(6);
    nh.getParam("cam0/intrinsics", cam0_intrinsics_temp);
    left_info.K[0] = cam0_intrinsics_temp[0];
    left_info.K[1] = cam0_intrinsics_temp[1];
    left_info.K[2] = cam0_intrinsics_temp[2];
    left_info.K[3] = cam0_intrinsics_temp[3];
    left_info.K[4] = cam0_intrinsics_temp[4];
    left_info.K[5] = cam0_intrinsics_temp[5];

    vector<double> cam1_intrinsics_temp(6);
    nh.getParam("cam1/intrinsics", cam1_intrinsics_temp);
    right_info.K[0] = cam1_intrinsics_temp[0];
    right_info.K[1] = cam1_intrinsics_temp[1];
    right_info.K[2] = cam1_intrinsics_temp[2];
    right_info.K[3] = cam1_intrinsics_temp[3];
    right_info.K[4] = cam1_intrinsics_temp[4];
    right_info.K[5] = cam1_intrinsics_temp[5];

    cv::Mat T_cam0_cam1 = utils::getTransformCV(nh, "cam1/T_cn_cnm1");
    right_info.P[0] = T_cam0_cam1.at<double>(0, 0);
    right_info.P[1] = T_cam0_cam1.at<double>(0, 1);
    right_info.P[2] = T_cam0_cam1.at<double>(0, 2);
    right_info.P[3] = T_cam0_cam1.at<double>(0, 3);
    right_info.P[4] = T_cam0_cam1.at<double>(1, 0);
    right_info.P[5] = T_cam0_cam1.at<double>(1, 1);
    right_info.P[6] = T_cam0_cam1.at<double>(1, 2);
    right_info.P[7] = T_cam0_cam1.at<double>(1, 3);
    right_info.P[8] = T_cam0_cam1.at<double>(2, 0);
    right_info.P[9] = T_cam0_cam1.at<double>(2, 1);
    right_info.P[10] = T_cam0_cam1.at<double>(2, 2);
    right_info.P[11] = T_cam0_cam1.at<double>(2, 3);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiscamera_ros");
    ros::NodeHandle nh_local("~");

    // Loading ROS Paramters
    loadCalibration(ros::NodeHandle("~/calibration")); //  write to left_info, right_info


    // Initialise publishers
    _pub = new StereoCameraPublisher(nh_local);
    _pub_imu = new IMUPublisher(nh_local);

    // list of cameras

    std::vector<std::unique_ptr<TisCameraManager>> camera_list;

    // camera_ns_list.push_back("30914056"); // left
    // camera_ns_list.push_back("30914060"); // right


    // sync struct
    int RATE;
    nh_local.getParam("sync_rate", RATE);
    // constexpr int RATE = 4; // should be divisible by 800
    
    CameraIMUSyncN cameraImuSync;
    cameraImuSync.set_max_slack(1.0/RATE * 0.8); // maximum slack to be 80% of the duration
    cameraImuSync.set_imu_read_jitter(0.002); // 2ms jitter allowed

    // GStreamer must be initialised before any camera usage
    gst_init(&argc, &argv);

    try {
        // nh_local.setParam("sync_rate", RATE);
        imu_vn_100::ImuVn100 imu(nh_local);
        imu.Stream(true);
        imu.registerCallback(std::bind(&CameraIMUSyncN::push_backIMU, &cameraImuSync, std::placeholders::_1));
        
        // Initialise available camera list

        auto available_devices = gsttcam::get_device_list();

        std::vector<std::string> camera_ns_list = CameraParam::loadCameras(nh_local);
        const int N = camera_ns_list.size();
        assert(N == 2); // ONLY STEREO FOR NOW

        size_t i = 0;
        for (auto &camera_ns : camera_ns_list)
        {
            CameraParam param;
            param.loadParam(camera_ns);
            param.loadExposureControlParam(param.type);

            auto& sn = param.camera_sn;
            // Skip non-existent cameras
            if (
                available_devices.end() == std::find_if(available_devices.begin(), available_devices.end(), [&sn](gsttcam::CameraInfo& device){ return sn == device.serial;})
            )
            {
                ROS_WARN_STREAM(camera_ns << "is not detected, skipping");
                camera_list.push_back(std::unique_ptr<TisCameraManager>());
                assert(camera_list.back() == nullptr);
                continue;
            }

            std::unique_ptr<TisCameraManager> camera(new TisCameraManager (camera_ns, sn));
            // todo: check for successful initialisation
            
            // std::cout << "Enable Display" << std::endl;
            // camera->enable_video_display(gst_element_factory_make("ximagesink", NULL));
            std::cout << "Setting Capture Format..." << std::endl;
            camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{param.width,param.height}, gsttcam::FrameRate{param.gst_frame_rate,1}); // {1440,1080}
            
            if (param.exposure_mode == "manual")
                camera->set_exposure_gain_auto(false);
            else
                camera->set_exposure_gain_auto(true);
            
            camera->set_exposure_time(param.initial_exposure); // in us
            camera->set_gain(param.initial_gain);

            if (param.hardware_sync_mode == "none")
            {
                camera->set_trigger_mode(TisCameraManager::NONE); // prior to start, the camera has to be non-triggering mode
                camera->start();
            }else if (param.hardware_sync_mode == "slave"){
                
                camera->set_trigger_mode(TisCameraManager::NONE); // prior to start, the camera has to be non-triggering mode
                camera->start();
                std::cout << "Camera " << camera_ns << " Hardware Sync ENABLED" << std::endl;
                camera->set_trigger_mode(TisCameraManager::TRIGGER_RISING_EDGE);
            }

            ROS_INFO_STREAM("Camera " << camera_ns << " Started...");            

            camera_list.push_back(std::move(camera));

            MASK |= 1<<i;
            i++;
        }

        // std::cout  << " MASK=" << (int)MASK << std::endl;

        cameraImuSync.set_camera_mask(MASK);

        cameraImuSync.register_callback_synced(callbackSynced_handler);

        ros::Duration(0.5).sleep();

        i = 0;
        for (auto& camera : camera_list){
            if (camera != nullptr)
                camera->registerCallback(std::bind(&CameraIMUSyncN::push_backCamera, &cameraImuSync, std::placeholders::_1, i));
            i++;
        }

        ros::spin();

        for (auto& camera : camera_list)
            camera->stop();

    } catch (const std::exception& e) {
        ROS_INFO("%s: %s", nh_local.getNamespace().c_str(), e.what());
    }

    return 0;
}