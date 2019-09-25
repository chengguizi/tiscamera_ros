/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-17 11:39:39 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2019-09-25 15:57:07
 */

#include <tiscamera_interface/tiscamera_interface.hpp>
#include <gst/gst.h>

#include <unistd.h>

#include <thread>
#include <vector>

#include <mutex>

#include <ros/ros.h>
#include <imu_vn_100/imu_vn_100.h>



#include "camera_imu_sync.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiscamera_ros");
    ros::NodeHandle nh_local("~");

    // list of cameras

    std::vector<std::unique_ptr<TisCameraManager>> camera_list;

    std::vector<std::string> camera_ns_list;

    camera_ns_list.push_back("30914056"); // left
    camera_ns_list.push_back("30914060"); // right


    // sync struct
    constexpr int RATE = 25;
    typedef CameraIMUSync<VnDeviceCompositeData, TisCameraManager::FrameData, 2> CameraIMUSyncN;
    CameraIMUSyncN cameraImuSync;
    cameraImuSync.set_camera_rate(1.0/RATE);

    gst_init(&argc, &argv);

    try {
        nh_local.setParam("sync_rate", RATE);
        imu_vn_100::ImuVn100 imu(nh_local);
        imu.Stream(true);
        imu.registerCallback(std::bind(&CameraIMUSyncN::push_backIMU, &cameraImuSync, std::placeholders::_1));
        

        std::size_t i = 0;
        for (auto& camera_ns : camera_ns_list)
        {
            std::unique_ptr<TisCameraManager> camera(new TisCameraManager ("tiscam" + std::to_string(i), camera_ns));
            // todo: check for successful initialisation
            

            camera->enable_video_display(gst_element_factory_make("ximagesink", NULL));
            camera->set_capture_format("GRAY16_LE", gsttcam::FrameSize{1440,1080}, gsttcam::FrameRate{30,1});
            camera->set_trigger_mode(TisCameraManager::NONE);
            camera->start();
            ROS_INFO_STREAM("Camera " << camera_ns << " Started...");
            camera->set_trigger_mode(TisCameraManager::TRIGGER_RISING_EDGE);

            camera->registerCallback(std::bind(&CameraIMUSyncN::push_backCamera, &cameraImuSync, std::placeholders::_1, i));

            camera_list.push_back(std::move(camera));

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