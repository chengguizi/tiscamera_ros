/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-13 14:33:01 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2019-09-24 18:01:56
 */

#include "tiscamera_interface.hpp"
#include "tiscamera_interface/tcam_statistics_meta.h"

#include <chrono>
#include <iostream>

#include <cassert>
#include <cstring>

using namespace gsttcam;

TisCameraManager::TisCameraManager(const std::string topic_ns, const std::string serial) : TcamCamera(serial)
{
    frame.data.camera_sn = serial;
    frame.data.topic_ns = topic_ns;
    
    frame.data.frame_count = 0;
    frame.data.image_data = nullptr;

    prop_trigger_mode = get_property("Trigger Mode");
    prop_exposure_mode = get_property("Exposure Auto");
    prop_exposure_time = get_property("Exposure Time (us)");
    prop_gain_mode = get_property("Gain Auto");
    prop_gain = get_property("Gain");

    std::cout << "Tiscamera initialised" << std::endl;
}

TisCameraManager::~TisCameraManager()
{

    // make sure all threads are shutdown, mutex released
    if(frame.data.image_data)
        delete frame.data.image_data;

    std::cout << "Tiscamera Destructor()" << std::endl;
}

bool TisCameraManager::set_trigger_mode(TisCameraManager::TriggerMode value)
{

    if (value == TRIGGER_RISING_EDGE)
    {
        std::cout << "setting trigger mode to RISING EDGE" << std::endl;

        if (!prop_trigger_mode->set((*this), (int)true))
        {
            throw std::runtime_error("set trigger mode FAILED");
        }
            
    }else
    {
        std::cout << "setting trigger mode to NONE" << std::endl;

        if (!prop_trigger_mode->set((*this), (int)false))
        {
            throw std::runtime_error("set trigger mode FAILED");
        }
    }
    

    try
    {
        prop_trigger_mode = get_property("Trigger Mode");
    }
    catch(std::exception &ex)    
    {
        printf("Error %s : %s\n",ex.what(), "Trigger Mode");
        return false;
    }

    std::cout  << prop_trigger_mode->to_string() << std::endl;

    return false;
}

bool TisCameraManager::set_exposure_gain_auto(bool value)
{
    std::cout << "setting exposure & gain mode to " << value << std::endl;

    if (!prop_exposure_mode->set((*this), value))
    {
        throw std::runtime_error("set exposure mode FAILED");
    }

    if (!prop_gain_mode->set((*this), value))
    {
        throw std::runtime_error("set gain mode FAILED");
    }

    return true;
}

bool TisCameraManager::set_exposure_time(int value)
{
    std::cout << "setting exposure time to " << value << std::endl;

    if (!prop_exposure_time->set((*this), value))
    {
        throw std::runtime_error("set exposure time FAILED");
    }
    return true;
}

bool TisCameraManager::set_gain(int value)
{
    std::cout << "setting gain to " << value << std::endl;

    if (!prop_gain->set((*this), value))
    {
        throw std::runtime_error("set gain FAILED");
    }
    return true;
}

void TisCameraManager::set_capture_format(std::string format, FrameSize size, FrameRate framerate)
{
    TcamCamera::set_capture_format(format, size, framerate);
}


bool TisCameraManager::start()
{
    std::cout << frame.data.topic_ns <<": starting camera..." << std::endl;

    set_new_frame_callback(std::bind(&TisCameraManager::setFrame, this, std::placeholders::_1, std::placeholders::_2), &frame);

    return TcamCamera::start();
}

bool TisCameraManager::stop()
{
    std::cout << frame.data.topic_ns <<": stopping camera..." << std::endl;

    return TcamCamera::stop();
}


GstFlowReturn TisCameraManager::setFrame(GstAppSink *appsink, gpointer data)
{
    (void)data; // unused
    if(frame.mtx.try_lock())
    {
        const uint old_buffer_size = frame.data.width * frame.data.height * frame.data.bytes_per_pixel;

        // obtain buffer location
        GstSample *sample = gst_app_sink_pull_sample(appsink);
        GstBuffer *buffer = gst_sample_get_buffer(sample);

        //// THIS IS NULL
        // const GstStructure * sample_info =  gst_sample_get_info(sample);

        // obtain caps associated
        const GstCaps  *caps = gst_sample_get_caps(sample);
        const GstStructure * caps_structure = gst_caps_get_structure(caps, 0);

        frame.data.pixel_format = gst_structure_get_string(caps_structure, "format");

        if (frame.data.pixel_format == "GRAY16_LE")
            frame.data.bytes_per_pixel = 2;
        else if (frame.data.pixel_format == "GRAY_8")
            frame.data.bytes_per_pixel = 1;
        else
            throw std::runtime_error("Unkown pixel format" + frame.data.pixel_format);

        gst_structure_get_int(caps_structure, "width", &frame.data.width);
        gst_structure_get_int(caps_structure, "height", &frame.data.height);

        // std::cout  << "frame_width " << frame_width << ", frame_height " << frame_height << ", bytes_per_pixel " << bytes_per_pixel << std::endl;

        // Obtain metadata from Tiscamera specifics
        const GstMetaInfo * meta_info = gst_meta_get_info("TcamStatisticsMeta");
        assert(meta_info);

        TcamStatisticsMeta *meta = (TcamStatisticsMeta *) gst_buffer_get_meta(buffer, meta_info->api); 

        gst_structure_get_uint64(meta->structure, "frame_count", &frame.data.frame_count);
        gst_structure_get_uint64(meta->structure, "frames_dropped", &frame.data.frames_dropped); 
        // this is a monotonic clock, but not realtime
        gst_structure_get_uint64(meta->structure, "capture_time_ns", &frame.data.capture_time_ns);
        gst_structure_get_double(meta->structure, "framerate", &frame.data.framerate);
        gst_structure_get_boolean(meta->structure, "is_damaged", &frame.data.is_damaged);

        // uint64_t camera_time_ns;
        // if (gst_structure_get_uint64(meta->structure, "camera_time_ns", &camera_time_ns))
        //     std::cout << "camera_time_ns = " << camera_time_ns << std::endl;

            // gst_structure_set(struc,
            //               "frame_count", G_TYPE_UINT64, statistics->frame_count,
            //               "frames_dropped", G_TYPE_UINT64, statistics->frames_dropped,
            //               "capture_time_ns", G_TYPE_UINT64, statistics->capture_time_ns,
            //               "camera_time_ns", G_TYPE_UINT64, statistics->camera_time_ns,
            //               "framerate", G_TYPE_DOUBLE, statistics->framerate,
            //               "is_damaged", G_TYPE_BOOLEAN, statistics->is_damaged,
            //               nullptr);
            // frame copy to frame struct

        GstMapInfo info;
        gst_buffer_map(buffer, &info, GST_MAP_READ);

        // buffer should match caps
        assert( (int)info.size == frame.data.width * frame.data.height * frame.data.bytes_per_pixel);

        assert(info.data);

        if (frame.data.image_data == nullptr)
            frame.data.image_data = new unsigned char[info.size];
        else if (old_buffer_size!= info.size)
        {
            std::cout << "Detected change of buffer size from " << old_buffer_size << " to  " << info.size << std::endl;
            delete [] frame.data.image_data;
            frame.data.image_data = new unsigned char[info.size];
        }

        std::memcpy(frame.data.image_data, info.data, frame.data.width * frame.data.height * frame.data.bytes_per_pixel);
        
        // Calling Unref is important!
        gst_buffer_unmap (buffer, &info);
        gst_sample_unref(sample);

        std::cout << frame.data.topic_ns << " " << frame.data.capture_time_ns  << " frame " << frame.data.frame_count << std::endl;

        if (_cblist_camera.size())
        {
            for (auto& cb : _cblist_camera)
                cb(frame.data);
        }

        frame.mtx.unlock();
        frame.con.notify_all();
    }
    else
    {
        std::cerr << frame.data.topic_ns << "Missed Frame " << frame.data.frame_count << std::endl;
    }

    return GST_FLOW_OK;
}

// should run on a separate thread
void TisCameraManager::processFrame()
{
    std::cout << frame.data.topic_ns <<  ": process frame loop starts..." << std::endl;
    while (is_playing)
    {
        std::unique_lock<std::mutex> lk(frame.mtx); // this call also locks the thread, with blocking behaviour
        auto ret = frame.con.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

        if (ret == std::cv_status::timeout ){
            std::cerr << frame.data.topic_ns << ": Wait timeout for new frame arrival..." << std::endl;
            continue;
        }

        // frame.data.image_data = nullptr; // make the next frame to take a new memory space
    }

    std::cout << frame.data.topic_ns <<  ": process frame loop terminates..." << std::endl;
}

void TisCameraManager::registerCallback(TisCameraManager::callbackCamera cb)
{
    _cblist_camera.push_back(cb);
}


TisCameraManager::FrameData TisCameraManager::getNextFrame()
{
    std::unique_lock<std::mutex> lk(frame.mtx); // this call also locks the thread, with blocking behaviour
    auto ret = frame.con.wait_for(lk,std::chrono::seconds(2)); // with ~0.03ms delay, lock reacquired

    if (ret == std::cv_status::timeout ){
        std::cerr << frame.data.topic_ns << ": Wait timeout for new frame arrival..." << std::endl;
        return FrameData();
    }

    FrameData ret_data = frame.data; 
    frame.data.image_data = nullptr; // make the next frame to take a new memory space

    return ret_data;
}