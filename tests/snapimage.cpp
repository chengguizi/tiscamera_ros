////////////////////////////////////////////////////////////////////////
/* Simple snapimage Example
This sample shows, how to snap a image from the live stream.

It uses the the examples/cpp/common/tcamcamera.* files as wrapper around the
GStreamer code and property handling. Adapt the CMakeList.txt accordingly.

As sample image processing an OpenCV cv::Mat is generated and saved as JPEG
*/

#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include <string>
 
#include "tiscamera_interface/tiscamera_interface.hpp"

#include "opencv2/opencv.hpp"

#include <iostream>

using namespace gsttcam;

// debuging for OS processing latency
struct timespec time_now;
uint64_t monotonic_time;


void ListProperties(TcamCamera &cam)
{
    // Get a list of all supported properties and print it out
    auto properties = cam.get_camera_property_list();
    std::cout << "Properties:" << std::endl;
    for(auto &prop : properties)
    {
        std::cout << prop->to_string() << std::endl;
    }
}

bool take_next = false;
TisCameraManager* cam = nullptr;

void imageCallback(std::shared_ptr<TisCameraManager::FrameData> data){

    // static int last_remainder = 0;
    static int total_count = 1;

    const int error = clock_gettime(CLOCK_MONOTONIC, &time_now);
    assert(error == 0);
    monotonic_time = (time_now.tv_sec * 1e9) + (time_now.tv_nsec);

    double delay_ms = (monotonic_time - data->get_info().capture_time_ns) / 1.0e6;

    std::cout << "OS delay = " << delay_ms << " ms" << std::endl;

    if (take_next){

        auto datainfo = data->get_info();
        std::cout << datainfo.frame_count << "taken." << std::endl;

        cv::Mat OpenCVImage;
        OpenCVImage.create( datainfo.height,datainfo.width,CV_16UC1);
        memcpy( OpenCVImage.data, data->image_data(), datainfo.bytes_per_pixel* datainfo.height * datainfo.width);
        std::ostringstream filename;

        // test for bit range
        // for

        filename << total_count << "-" << datainfo.frame_count << ".tiff";

        cv::imwrite(filename.str() ,OpenCVImage);
        take_next = false;
        total_count++;
    }

    data->release();
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);

    std::string camera_sn = "30914056"; //"30914056" "30914060"

    cam = new TisCameraManager("snapcam",camera_sn);

    std::cout << "start camera " << camera_sn << std::endl;
    

    // Comment following line, if no live video display is wanted.
    cam->enable_video_display(gst_element_factory_make("xvimagesink", NULL)); // ximagesink
    
    // Set a color video format, resolution and frame rate
    cam->set_capture_format("GRAY16_LE", FrameSize{1440,1080}, FrameRate{30,1});
    // cam.set_capture_format("GRAY16_LE", FrameSize{640,480}, FrameRate{30,1});

    // Start the camera
    cam->set_trigger_mode(TisCameraManager::NONE);
    cam->set_exposure_gain_auto(false);
    cam->set_exposure_time(3000); // in us
    cam->set_gain(0);

    // cam->set_tonemapping_mode(true);

    cam->start();

    ListProperties(*cam);

    cam->registerCallback(imageCallback);

    printf("You may start Snap now\n");

    // char dummyvalue;

    while(true){
        // printf("Press space key to take Snap.\n");
        std::cin.get();

        take_next = true;
        
    }

    cam->stop();

    return 0;
}