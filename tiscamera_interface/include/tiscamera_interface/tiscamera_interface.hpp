#ifndef TISCAMERA_INTERFACE_HPP
#define TISCAMERA_INTERFACE_HPP

#include "tcamcamera.h"

#include <mutex>
#include <condition_variable>
#include <vector>

#include <memory>


class TisCameraManager : public gsttcam::TcamCamera
{
    public:

        enum TriggerMode{
            NONE = 0,
            TRIGGER_RISING_EDGE,
            TRIGGER_FALLING_EDGE
        };

        struct FrameData{

            // Static info
            std::string topic_ns;
            std::string camera_sn;
            int width;
            int height;
            int bytes_per_pixel;
            std::string pixel_format;

            // Gstreamer Metadata
            uint64_t frame_count;
            uint64_t frames_dropped;
            uint64_t capture_time_ns;
            // uint64_t camera_time_ns;
            double framerate;
            int is_damaged; // gboolean is implemented as int

            // payload
            unsigned char *image_data;

            void release(){if (image_data) delete [] image_data;};
        };

        typedef std::function<void(const FrameData&)> callbackCamera;

        TisCameraManager(const std::string topic_ns, const std::string serial = "");
        ~TisCameraManager();

        bool set_trigger_mode(TriggerMode value);
        bool set_low_latency_mode(bool value);

        void set_capture_format(std::string format, gsttcam::FrameSize size, gsttcam::FrameRate framerate);
        bool start(); // gst playing state
        bool stop();
        void processFrame(); // run in a separate thread
        FrameData getNextFrame();

        void registerCallback(callbackCamera cb);
    
    private:
        bool is_streaming;
        struct FrameDataMutexed{
            FrameData data;

            std::mutex mtx;
            std::condition_variable con;
        }frame;

        // below are the mutex protected functions
        GstFlowReturn setFrame(GstAppSink *appsink, gpointer data); // callback from the Gstreamer backend

        // properties
        std::shared_ptr<gsttcam::Property> prop_trigger_mode;

        std::vector<callbackCamera> _cblist_camera;

};

#endif /* TISCAMERA_INTERFACE_HPP */
