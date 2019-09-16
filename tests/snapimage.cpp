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
 
#include "tcamimage.h"

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

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);

    cv::Mat OpenCVImage;
    // Initialize our TcamCamera object "cam" with the serial number
    // of the camera, which is to be used in this program.
    TcamImage cam;
    // TcamImage cam("30914056");
    //TcamCamera cam("00001234");

    ListProperties(cam);

    // Comment following line, if no live video display is wanted.
    cam.enable_video_display(gst_element_factory_make("ximagesink", NULL));
    
    // Set a color video format, resolution and frame rate
    cam.set_capture_format("GRAY16_LE", FrameSize{640,480}, FrameRate{30,1});


    // sleep(1);
    // return 0;
    
    // Start the camera
    cam.start();

    // sleep(1);
    printf("Start Snap\n");

    std::cout  << " cam.getBytesPerPixel() = " << cam.getBytesPerPixel() << std::endl;

    // Snap an Image with 60 ms timeout. Should be set accordingly to the
    // frame rate.
    if( cam.snapImage(500) )
    {
        // On succes do something with the image data. Here we create
        // a cv::Mat and save the image
        // OpenCVImage.create( cam.getHeight(),cam.getWidth(),CV_16UC1);
        // memcpy( OpenCVImage.data, cam.getImageData(), cam.getImageDataSize());
        // cv::imwrite("test.png",OpenCVImage);
    }
    else
    {
        printf("Timeout at snapImage()\n");
    }
    sleep(3);
    cam.set_capture_format("GRAY16_LE", FrameSize{1440,1080}, FrameRate{30,1});

    printf("Press enter key to end program.");
    // Simple implementation of "getch()", wait for enter key.
    char dummyvalue[10];
    scanf("%c",dummyvalue);

    cam.stop();

    return 0;
}