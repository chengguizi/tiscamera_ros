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

// for tonemapping, dependency cvl-mirror repo on GitHub
#ifdef TONEMAPPING
#include <cvl/cvl.h>
#endif

using namespace std;
typedef CameraIMUSync<VnDeviceCompositeData, TisCameraManager::FrameData, 2> CameraIMUSyncN;
unsigned char MASK = 0;


StereoCameraPublisher* _pub;
IMUPublisher* _pub_imu;
sensor_msgs::CameraInfo left_info, right_info;

std::vector<std::string> CameraParam::camera_list;



void ListProperties(TisCameraManager &cam)
{
    // Get a list of all supported properties and print it out
    auto properties = cam.get_camera_property_list();
    std::cout << "Properties:" << std::endl;
    for(auto &prop : properties)
    {
        std::cout << prop->to_string() << std::endl;
    }
}


/*

Handling High Dynamic Range (HDR) frames.
All tone mapping operators (TMOs) take normalized CVL_XYZ frames as input. 
If you have CVL_XYZ frames with absolute values, you need to convert them with cvl_luminance_range() first. 

TMOs that work on absolute luminance values have an additional parameter called max_abs_lum. 
If the maximum absolute luminance is known, this parameter can be set accordingly. 
If the maximum absolute luminance is not known, the parameter can be used to prescale the values to a suitable range.

*/

// to memory copy the data from srcptr_raw to frame, and convert to rgb (xyz) float, before processing
cv::Mat do_tonemapping(cvl_frame_t* frame, void* srcptr_raw, float max_abs_lum)
{

    //// declare a temperary frame to pass to the tonemapping function call
    static cvl_frame_t *_tmpframe = nullptr, *_frame1 = nullptr;
    if(!_tmpframe)
    {
    _tmpframe = cvl_frame_new(
        cvl_frame_width(frame), cvl_frame_height(frame), 
        4, CVL_UNKNOWN, CVL_FLOAT16, CVL_MEM);
    }


    auto mat = cv::Mat(cv::Size(cvl_frame_width(frame), cvl_frame_height(frame)), CV_16UC1);

    //// memory copy from raw buffer to a frame


    //// convert greyscale to rgb (xyz)
    // cv::cvtColor(src, src, cv::COLOR_GRAY2RGB);
    // assert(src.isContinuous());

    //// fill up frame's data from opencv

    

    std::cout << "do_tonemapping()" << std::endl;
    float *p = static_cast<float *>(cvl_frame_pointer(frame));
    uint16_t *srcptr = static_cast<uint16_t *> (srcptr_raw);
    
    std::cout << "img size = " << cvl_frame_width(frame) * cvl_frame_height(frame) << std::endl;

    double time_start = ros::Time::now().toSec();
    
    for (int i = 0; i < cvl_frame_width(frame) * cvl_frame_height(frame); i++)
    {
        float scaled = srcptr[i] / max_abs_lum;
        p[3 * i + 0] = scaled;
        p[3 * i + 1] = scaled;
        p[3 * i + 2] = scaled;
    }

    double time_end = ros::Time::now().toSec();

    

    float min_abs_lum;
    cvl_reduce(frame, CVL_REDUCE_MIN, 1, &min_abs_lum);
    min_abs_lum *= max_abs_lum;

    min_abs_lum = std::max(0.00001f, min_abs_lum);	
    max_abs_lum = std::max(min_abs_lum, max_abs_lum);


    std::cout << "frame min max:"<< min_abs_lum << ", " << max_abs_lum << std::endl;

    float threshold = 5; // [0,10]

    cvl_frame_free(_frame1);
    _frame1 = cvl_frame_new(cvl_frame_width(frame), cvl_frame_height(frame),
    		    3, CVL_XYZ, CVL_FLOAT16, CVL_TEXTURE);

    

    cvl_tonemap_ashikhmin02(_frame1, frame, 
		    min_abs_lum, max_abs_lum,
		    _tmpframe, threshold);

    

    float ret_min, ret_max;
    cvl_reduce(_frame1, CVL_REDUCE_MIN, 1, &ret_min);
    cvl_reduce(_frame1, CVL_REDUCE_MAX, 1, &ret_max);

    std::cout << "frame1 min max:"<< ret_min << ", " << ret_max << std::endl;

    
    
    

    

    p = static_cast<float *>(cvl_frame_pointer(_frame1));
    for (int i = 0; i < cvl_frame_width(frame) * cvl_frame_height(frame); i++)
    {
        mat.at<ushort>(i) = (uint16_t)(p[3 * i] * max_abs_lum);
    }

    

    std::cout << "time for tonemapping = "<< 1000 * (time_end - time_start) << " msecs" << std::endl;

    return mat;
}

// template <class TimuData, class TcameraData, int Ncamera>
void callbackSynced_handler(CameraIMUSyncN::MetaFrame frame)
{

    // following code from /cvl-mirror/cvtool/doc/cvl.html
    static cvl_gl_context_t *gl_context = nullptr;
    static cvl_frame_t* _tmpframe = nullptr;
    // cvl_frame_t *input_frame, *output_frame;

    // cvl_frame_t * _tmpframe = cvl_frame_new(20, 10, 
    //         3, CVL_XYZ, CVL_FLOAT16, CVL_MEM);

    
    // convert data to cv::Mat

    auto& left_frame = frame.camera[0];
    auto& right_frame = frame.camera[1];

    if (!_tmpframe)
    {
        /* Create a GL context on display ":0" and activate it. */
        gl_context = cvl_gl_context_new(":0");
        if (!gl_context)
        {
            fprintf(stderr, "Cannot create GL context.\n");
            abort();
        }

        cvl_init();

        std::cout << "initialising cvl buffer" << left_frame.width << ", " << left_frame.height << std::endl;
        _tmpframe = cvl_frame_new(left_frame.width, left_frame.height, 3, CVL_XYZ, CVL_FLOAT16, CVL_MEM); // OR CVL_FLOAT
    }

    // cv::Mat frame_ret_l = do_tonemapping(_tmpframe, left_frame.image_data, 65535);
    
    // auto left = frame_ret_l;
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
    cameraImuSync.set_imu_read_jitter(std::max(1.0/RATE * 0.1, 0.008)); // 8ms jitter allowed

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
            // camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{1440,1080}, gsttcam::FrameRate{30,1});
            camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{param.width,param.height}, gsttcam::FrameRate{param.gst_max_frame_rate,1}); // {1440,1080}
            
            if (param.exposure_mode == "manual")
            {
                std::cout << "Setting Exposure Mode Manual" << std::endl;
                camera->set_exposure_gain_auto(false);
                camera->set_exposure_time(param.initial_exposure); // in us
                camera->set_gain(param.initial_gain);
            }
            else if (param.exposure_mode == "internal")
            {
                std::cout << "Setting Exposure Mode Auto" << std::endl;
                camera->set_exposure_gain_auto(true);
            }else
            {
                throw std::runtime_error("Unknown Exposure Mode");
            }
            
            

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

            // POST START PARAMETERS
            //// tonemapping parameters
            if (param.tonemapping_mode == "internal")
            {
                camera->set_tonemapping_mode(true);
                camera->set_tonemapping_param(param.tonemapping_intensity, param.tonemapping_global_brightness);

            }else{
                camera->set_tonemapping_mode(false);
            }

            //// gamma
            camera->set_gamma(param.initial_gamma);

            //// highlight reduction
            camera->set_highlight_reduction(param.highlight_reduction);

            //// auto exposure settings
            camera->set_exposure_limits(param.auto_upper_limit_exposure, param.lower_limit_exposure, param.upper_limit_exposure);
            camera->set_gain_limits(param.lower_limit_gain, param.upper_limit_gain);

            camera->set_exposure_auto_reference(param.exposure_auto_reference);

            ListProperties(*camera);
            

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