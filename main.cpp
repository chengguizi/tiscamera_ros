/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-17 11:39:39 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-07-02 19:25:30
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
#include "camera_trigger_subscriber.hpp"

#include "ros_publisher.hpp"
#include "utils.hpp"

#include "camera_param.hpp"

// for tonemapping, dependency cvl-mirror repo on GitHub
#ifdef TONEMAPPING
#include <cvl/cvl.h>
#endif


typedef CameraIMUSync<TisCameraManager::FrameData> CameraIMUSyncN;
unsigned char MASK = 0; // currently, maximum 8 cameras

// publisher for individual camera, if they are free running (not synced); otherwise nullptr
std::vector<std::shared_ptr<ImagePublisher>> _camera_pub;
std::shared_ptr<IMUPublisher> _pub_imu;


// it also store the camera_ns string and hardware_sync_mode variable for each camera
std::vector<std::shared_ptr<CameraParam>> CameraParam::list;


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

#ifdef TONEMAPPING
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
#endif

inline void publish_image(std::shared_ptr<TisCameraManager::FrameData> frame, size_t index, uint64_t trigger_time)
{

    assert(frame->initialised());

    //// following code from /cvl-mirror/cvtool/doc/cvl.html
    // static cvl_gl_context_t *gl_context = nullptr;
    // static cvl_frame_t* _tmpframe = nullptr;
    
    //// convert data to cv::Mat

    // if (!_tmpframe)
    // {
    //     /* Create a GL context on display ":0" and activate it. */
    //     gl_context = cvl_gl_context_new(":0");
    //     if (!gl_context)
    //     {
    //         fprintf(stderr, "Cannot create GL context.\n");
    //         abort();
    //     }

    //     cvl_init();

    //     std::cout << "initialising cvl buffer" << frame.width << ", " << frame.height << std::endl;
    //     _tmpframe = cvl_frame_new(left_frame.width, left_frame.height, 3, CVL_XYZ, CVL_FLOAT16, CVL_MEM); // OR CVL_FLOAT
    // }

    // cv::Mat frame_ret_l = do_tonemapping(_tmpframe, left_frame.image_data, 65535);
    
    // auto left = frame_ret_l;
    auto image_cv = cv::Mat(cv::Size(frame->width, frame->height), CV_16UC1, (void *)frame->image_data(), cv::Mat::AUTO_STEP);

    assert(_camera_pub[index] != nullptr);

    _camera_pub[index]->publish(image_cv, "mono16", CameraParam::list[index]->camera_info, ros::Time().fromNSec(trigger_time));
}

void callbackIndividual_handler(std::shared_ptr<TisCameraManager::FrameData> data, const size_t index)
{
    // use capture time as the 'trigger time', as the camera is free running
    publish_image(data, index, data->capture_time_ns);
    data->release();
}


void callbackSynced_handler(const CameraIMUSyncN::MetaFrame& frame)
{
    for (size_t i = 0; i < frame.camera.size(); i++){

        // skip cameras that are not in the sync group
        if (frame.cameraBitMask & 1<<i){
            publish_image(frame.camera[i], i, frame.trigger_time);
            frame.camera[i]->release();
        }
            
    }
}




void start_camera(std::unique_ptr<TisCameraManager>& camera, const CameraParam& param)
{
    // todo: check for successful initialisation
    
    // std::cout << "Enable Display" << std::endl;
    // camera->enable_video_display(gst_element_factory_make("ximagesink", NULL));
    std::cout << "Setting Capture Format..." << std::endl;
    // camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{1440,1080}, gsttcam::FrameRate{30,1});
    camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{param.width,param.height}, gsttcam::FrameRate{param.gst_max_frame_rate,1}); // {1440,1080}
    
    camera->set_imx_low_latency_mode(true);
    
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
        std::cout << "Camera " << param.camera_ns << " Hardware Sync ENABLED" << std::endl;
        camera->set_trigger_mode(TisCameraManager::TRIGGER_RISING_EDGE);
    }else if (param.hardware_sync_mode == "master"){
        camera->set_trigger_mode(TisCameraManager::NONE); // prior to start, the camera has to be non-triggering mode
        camera->start();
    }else
    {
        throw std::runtime_error("Unknown hardware_sync_mode");
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
    

    ROS_INFO_STREAM("Camera " << param.camera_ns << " Started...");
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "tiscamera_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    // Initialise publishers

    _pub_imu.reset(new IMUPublisher(nh_local));

    // store the list of cameras, those failed to initialise will have nullptr
    std::vector<std::unique_ptr<TisCameraManager>> tiscamera_list;


    // sync struct
    int RATE;
    std::string IMU_SOURCE;
    nh_local.getParam("sync_rate", RATE);
    nh_local.getParam("imu_source", IMU_SOURCE);
    ROS_INFO_STREAM("IMU sync rate (slave mode) is " << RATE << "fps");
    ROS_INFO_STREAM("IMU source is from" << IMU_SOURCE);


    // GStreamer must be initialised before any camera usage
    gst_init(&argc, &argv);

    bool has_master = false;
    bool has_slave = false;

    std::shared_ptr<CameraIMUSyncN> cameraImuSync;
    std::shared_ptr<imu_vn_100::ImuVn100> imu_vn100;
    std::shared_ptr<CamIMUStampSubscriber> imu_mavros;

    std::cout << "Discovering Cameras..." << std::endl;

    try {

        // Initialise available camera list

        auto available_devices = gsttcam::get_device_list();

        CameraParam::loadCameras(nh_local);
        const unsigned int N = CameraParam::list.size();
        assert(N <= 8);

        _camera_pub.resize(N);

        cameraImuSync.reset(new CameraIMUSyncN(N));
        cameraImuSync->set_max_slack(1.0/RATE * 0.8); // maximum slack to be 80% of the duration
        cameraImuSync->set_imu_read_jitter(std::max(1.0/RATE * 0.1, 0.008)); // 8ms jitter allowed

        
        for (size_t i = 0; i < CameraParam::list.size(); i++)
        {
            auto &param = CameraParam::list[i];
            auto &pub = _camera_pub[i];

            std::cout << std::endl << "Loading Camera No. " << i  << ": " << param->camera_ns << std::endl;
            pub.reset();
            param->loadCalibration(ros::NodeHandle("~/calibration")); //  write to left_info, right_info
            param->loadParam(param->camera_ns);
            param->loadExposureControlParam(param->type);

            auto& sn = param->camera_sn;
            // Skip non-existent cameras
            if (
                available_devices.end() == std::find_if(available_devices.begin(), available_devices.end(), [&sn](gsttcam::CameraInfo& device){ return sn == device.serial;})
            )
            {
                ROS_WARN_STREAM(param->camera_ns << "is not detected, skipping");
                tiscamera_list.push_back(std::unique_ptr<TisCameraManager>());
                assert(tiscamera_list.back() == nullptr);
                continue;
            }

            

            std::unique_ptr<TisCameraManager> camera(new TisCameraManager (param->camera_ns, sn));
            start_camera(camera, *param);

            camera->camera_ns = param->camera_ns; // for convenience

            tiscamera_list.push_back(std::move(camera));

            // if it is free running, do not add it into the sync system
            if (param->hardware_sync_mode != "none")
                MASK |= 1<<i;


            pub.reset( new ImagePublisher(nh_local, param->camera_ns));
        }

        std::cout << "Discovering Cameras [DONE]" << std::endl;


        cameraImuSync->set_camera_mask(MASK);
        std::cout  << " MASK=" << (int)MASK << std::endl;
        
        cameraImuSync->register_callback_synced(callbackSynced_handler);

        ros::Duration(0.5).sleep();

        std::cout << "Registering Callbacks..." << std::endl;

        for (size_t i = 0; i < CameraParam::list.size(); i++){
            
            auto& camera = tiscamera_list[i];
            auto& param = CameraParam::list[i];

            
            
            if (camera != nullptr){
                std::cout << "processing " << param->camera_ns << std::endl;
                assert(param->camera_ns == camera->camera_ns);
                if (param->hardware_sync_mode == "none")
                {
                    // here we should give callback to individual ros publish topics.
                    camera->registerCallback(std::bind(callbackIndividual_handler, std::placeholders::_1, i));
                }
                else if (param->hardware_sync_mode == "slave")
                {
                    has_slave = true;
                    camera->registerCallback(std::bind(&CameraIMUSyncN::push_backCamera, cameraImuSync, std::placeholders::_1, i, false));
                }
                else if (param->hardware_sync_mode == "master")
                {
                    has_master = true;
                    // there should be only 1 master at most, the master will act as a fake IMU
                    camera->registerCallback(std::bind(&CameraIMUSyncN::push_backCamera, cameraImuSync, std::placeholders::_1, i, true));
                }
                else
                {
                    throw std::runtime_error("unkown hardware_sync_mode");
                }
                
            }else{
                std::cout << "skipping " << param->camera_ns << std::endl;
            }

        }

        std::cout << "Registering Callbacks [Done]" << std::endl;

        //// Launch external IMU if needed
        if (has_slave && !has_master){
            std::cout << "launching external IMU unit " << IMU_SOURCE << std::endl;

            if(IMU_SOURCE == "vn-100"){
                imu_vn100.reset(new imu_vn_100::ImuVn100(nh_local));
                imu_vn100->Stream(true);
                imu_vn100->registerCallback(std::bind(&CameraIMUSyncN::push_backIMU, cameraImuSync, std::placeholders::_1, std::placeholders::_2));
            }
            else if(IMU_SOURCE == "mavros"){
                imu_mavros.reset(new CamIMUStampSubscriber(nh,"/mavros/cam_imu_sync/cam_imu_stamp"));
                imu_mavros->registerCallback(std::bind(&CameraIMUSyncN::push_backIMU, cameraImuSync, std::placeholders::_1, std::placeholders::_2));

            }else{
                throw std::runtime_error("Unkown IMU_SOURCE type");
            }
            
        }else{
            std::cout << "skipping launching external IMU unit" << std::endl;
        }

        ros::spin();

        for (auto& camera : tiscamera_list)
            camera->stop();

    } catch (const std::exception& e) {
        ROS_INFO("%s: %s", nh_local.getNamespace().c_str(), e.what());
    }

    

    

    return 0;
}