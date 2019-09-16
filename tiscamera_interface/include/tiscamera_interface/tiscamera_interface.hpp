#ifndef TISCAMERA_INTERFACE_HPP
#define TISCAMERA_INTERFACE_HPP

#include "tcamcamera.h"

#include <mutex>
#include <condition_variable>
#include <vector>


class TisCameraManager : public gsttcam::TcamCamera
{
    public:
        struct FrameData{

            // Static info
            uint32_t width;
            uint32_t height;
            std::string pixel_format;

            // Gstreamer Metadata
            uint64_t frame_count;
            uint64_t frames_dropped;
            uint64_t capture_time_ns;
            uint64_t camera_time_ns;
            double framerate;
            bool is_damaged;

            // payload
            unsigned char *image_data;
        };

        TisCameraManager(const std::string serial = "");
        ~TisCameraManager();

        void set_capture_format(std::string format, gsttcam::FrameSize size, gsttcam::FrameRate framerate);
        bool start(); // gst playing state
        bool stop(); // gst null state
    
    private:
        struct FrameDataMutexed{
            FrameData data;

            std::mutex mtx;
            std::condition_variable con;
        }frame;

};

#endif /* TISCAMERA_INTERFACE_HPP */
