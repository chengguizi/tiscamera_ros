/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-24 14:45:57 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-06-26 17:34:08
 */
#ifndef CAMERA_IMU_SYNC_HPP
#define CAMERA_IMU_SYNC_HPP

#include <cmath>
#include <forward_list>

#include <mutex>

#include <functional>
#include <vector>

/**
 * Every IMU data coming in, will trigger a collection machanism, to collect incoming camera frames. 
 * The time stamps are strictly more recent, and no larger than the maximum duration
 * 
 * For simplicity and efficiency, here we assume IMU data always come in before all camera frames, in realtime
 */ 
template <class TcameraData>
class CameraIMUSync{
    public:

        struct MetaFrame{
            uint64_t trigger_time;
            std::vector<TcameraData> camera;
            unsigned char cameraBitMask;

            void reset(){cameraBitMask = 0;};
            bool isComplete(const unsigned char& CAMERA_MASK){return (cameraBitMask == CAMERA_MASK); };
        };

        CameraIMUSync(int Ncamera) : Ncamera(Ncamera){

            // initialise meta frame to have buffer size
            metaFrame.resize(BUFFER_SIZE);

            // initialise image buffers within each frame
            for (auto& f : metaFrame)
                f.camera.resize(Ncamera);

            // intialise masks
            CAMERA_MASK = std::pow(2, Ncamera) - 1;
        };
        void set_max_slack(double sec){max_slack = sec * 1e9;}; // maximum tolerated delay of camera respect to IMU
        void set_imu_read_jitter(double sec){max_imu_read_jitter = sec * 1e9;}; // maximum delay of imu buffer being read by kernel
        void set_camera_mask(const unsigned char camera_mask){CAMERA_MASK = camera_mask;};
        void push_backIMU(uint64_t monotonic_time, uint64_t timeSyncIn);
        void push_backCamera(const TcameraData& data, const uint index);
        
        typedef std::function<void(const MetaFrame&)> callbackSynced;
        void register_callback_synced(callbackSynced cb){_cblist_synced.push_back(cb); };

    
    private:

        unsigned int Ncamera;
        unsigned char CAMERA_MASK;

        const static uint BUFFER_SIZE = 16;
        unsigned char BUFFER_MASK = BUFFER_SIZE - 1;
        #define MASK(x) ((x) & BUFFER_MASK)

        uint64_t max_slack; // maximum duration an IMU will wait for a camera frame         
        uint64_t max_imu_read_jitter;
        std::mutex mtx;
        

        std::vector<MetaFrame> metaFrame;

        std::forward_list<std::function<void(const TcameraData&, const uint)>> cam_callback_cache;

        unsigned char begin = 0;
        unsigned char end = 0;

        std::vector<callbackSynced> _cblist_synced;
        void syncedCallback(const MetaFrame);
};

template <class TcameraData>
void CameraIMUSync<TcameraData>::push_backIMU(uint64_t monotonic_time, uint64_t timeSyncIn)
{
    // std::lock_guard<std::mutex> lock(mtx);

    mtx.lock();

    // push back the data
    MetaFrame& frame = metaFrame[MASK(end)];
    
    frame.reset();
    frame.trigger_time = monotonic_time - timeSyncIn;
    end++;

    std::cout << "IMU syncIn Time " <<  timeSyncIn << ", index " << MASK(end - 1) << std::endl;
    std::cout << "IMU " <<  monotonic_time << ", index " << MASK(end - 1) << std::endl;

    if(MASK(begin) == MASK(end)){ // remove the oldest record when full
        begin++;
    }

    mtx.unlock();

    // Execute cached camera callbacks
    while (!cam_callback_cache.empty()){
        std::cout << "Execute Camera Callback Cache" << std::endl;
        cam_callback_cache.front();
        cam_callback_cache.pop_front();
    }
}

template <class TcameraData>
void CameraIMUSync<TcameraData>::push_backCamera(const TcameraData& data, const uint index)
{
    std::lock_guard<std::mutex> lock(mtx);

    // std::cout << "push_backCamera() " << index << std::endl;

    assert(index < Ncamera); // must not be out of range
    assert(MASK(begin) != MASK(end));

    const uint64_t& camera_time = data.capture_time_ns;

    // start searching from the oldest imu readings, find the first imu reading that is within the max slack
    for (unsigned char i = MASK(begin); i != MASK(end); i = MASK(i+1))
    {
        MetaFrame& frame = metaFrame[i];
        // const uint64_t& imu_time = frame.imu.monotonic_time;
        // Use this to minus away the offset
        const uint64_t& imu_time = frame.trigger_time;

        // Conversion from unsigned to signed notation
        const double slack_sec = imu_time < camera_time ? 
            (camera_time - imu_time) / 1.0e9 : 
            - ((imu_time - camera_time) / 1.0e9) ;

        // If camera is later than the imu time with maximum slack, skip
        if (imu_time + max_slack < camera_time){

            // // if it is the latest imu already, means the camera callback arrived before imu callback
            // if (i == MASK(end - 1))
            // else
                continue;
        }
        
        // Reaching here, camera is within the slack time of the imu stamp

        // Now, checking if the camera is not too far ahead the current imu
        if ( imu_time - max_imu_read_jitter < camera_time){ // allow IMU to be slightly slower than camera, due to jitter
            
            std::cout << "Slack (ms) = " << slack_sec * 1e3 << ", Adding frame " << index << "( time " << camera_time << ") to IMU " << int(i) << "(" << imu_time << ")" << std::endl;
            const unsigned char bit = 1 << index;
            assert( (frame.cameraBitMask & bit) == 0);

            frame.cameraBitMask = frame.cameraBitMask | bit;
            // TODO: currently, this only works if the sync is completed, before the next batch of frames comes. But this is normally the case
            frame.camera[index] = data;

            // detect if bitMask is full
            if (frame.isComplete(CAMERA_MASK))
            {
                std::cout << "IMU sync complete! for index " << MASK(i) << std::endl << std::endl;

                if( i != MASK(begin)){
                    std::cerr << "[[[[[WARNING: Skipping " << MASK(i-begin) << " IMU data]]]]]" << std::endl;
                }

                begin = MASK(i+1);

                syncedCallback(frame);

                frame.reset();
            }
        }else{
            std::cerr << "||||||||Failed to add camera frame: " << camera_time << std::endl;
            std::cerr << "Slack (ms) = " << slack_sec * 1e3 << std::endl; 
        }

        return;
    }

    // Reaching here, means the camera callback reaches earlier than imu callback

    // std::function<void(const TcameraData&, const uint)> func = [](const TcameraData& data, const uint index)

    std::cout << "Adding camera frame " << index << " to cache..." << std::endl;
    cam_callback_cache.push_front(std::bind(&CameraIMUSync::push_backCamera, this, data, index));
}

template <class TcameraData>
void CameraIMUSync<TcameraData>::syncedCallback(MetaFrame frame){

    for (auto& cb : _cblist_synced){
        cb(frame);
    }
}


#endif /* CAMERA_IMU_SYNC_HPP */
