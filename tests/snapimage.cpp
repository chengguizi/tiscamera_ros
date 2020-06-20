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

void imageCallback(const TisCameraManager::FrameData& data){

    // static int last_remainder = 0;
    static int total_count = 1;
    

    if (take_next){
        std::cout << data.frame_count << "taken." << std::endl;

        cv::Mat OpenCVImage;
        OpenCVImage.create( data.height, data.width,CV_16UC1);
        memcpy( OpenCVImage.data, data.image_data, data.bytes_per_pixel*data.height*data.width);
        std::ostringstream filename;

        // test for bit range
        // for

        filename << total_count << "-" << data.frame_count << ".tiff";

        cv::imwrite(filename.str() ,OpenCVImage);
        take_next = false;
        total_count++;
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);


    cam = new TisCameraManager("30914056");

    ListProperties(*cam);

    // Comment following line, if no live video display is wanted.
    cam->enable_video_display(gst_element_factory_make("ximagesink", NULL));
    
    // Set a color video format, resolution and frame rate
    cam->set_capture_format("GRAY16_LE", FrameSize{1440,1080}, FrameRate{30,1});
    // cam.set_capture_format("GRAY16_LE", FrameSize{640,480}, FrameRate{30,1});

    // Start the camera
    cam->set_trigger_mode(TisCameraManager::NONE);
    cam->set_exposure_gain_auto(false);
    cam->set_exposure_time(10000); // in us
    cam->set_gain(200);

    // cam->set_tonemapping_mode(true);

    cam->start();

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