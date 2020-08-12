/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-24 14:45:57 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-06-27 19:45:54
 */
#ifndef CAMERA_IMU_SYNC_HPP
#define CAMERA_IMU_SYNC_HPP

#include <cmath>
#include <list>

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
            std::vector<std::shared_ptr<TcameraData>> camera;
            unsigned char cameraBitMask;

            void reset(){
                cameraBitMask = 0;
                for (auto& cam : camera){
                    if(cam){
                        std::lock_guard<std::recursive_mutex> lock(cam->mtx);
                        cam->release(); // de-initialise the data buffer, so can be over-written
                        cam = nullptr; // remove from our metaframe database
                    }
                }
            }
            bool isComplete(const unsigned char& CAMERA_MASK){return (cameraBitMask == CAMERA_MASK); };
        };

        CameraIMUSync(int Ncamera) : Ncamera(Ncamera), imu_initialised(false), camera_initialised(false){

            // initialise meta frame to have buffer size
            metaFrame.resize(BUFFER_SIZE);

            // initialise image buffers within each frame
            for (auto& f : metaFrame)
                f.camera.resize(Ncamera);

            // intialise masks
            CAMERA_MASK = std::pow(2, Ncamera) - 1;
        };
        void set_mean_delay(double sec){mean_delay = sec * 1e9;}; // the expected delay between imu trigger and data arrival of image frames in buffer
        void set_max_slack(double sec){max_slack = sec * 1e9;}; // maximum tolerated delay of camera respect to IMU
        void set_imu_read_jitter(double sec){max_imu_read_jitter = sec * 1e9;}; // maximum delay of imu buffer being read by kernel
        void set_camera_mask(const unsigned char camera_mask){CAMERA_MASK = camera_mask;};
        void push_backIMU(uint64_t monotonic_time, uint64_t timeSyncIn);
        void push_backCamera(std::shared_ptr<TcameraData> data, const uint index, bool master);
        
        typedef std::function<void(const MetaFrame&)> callbackSynced;
        void register_callback_synced(callbackSynced cb){_cblist_synced.push_back(cb); };

    
    private:

        unsigned int Ncamera;
        unsigned char CAMERA_MASK;

        const static uint BUFFER_SIZE = 8;
        unsigned char BUFFER_MASK = BUFFER_SIZE - 1;
        #define MASK(x) ((x) & BUFFER_MASK)

        uint64_t mean_delay;
        uint64_t max_slack; // maximum duration an IMU will wait for a camera frame, beyond the mean delay
        uint64_t max_imu_read_jitter; // ahead of the mean delay
        std::recursive_mutex mtx;
        

        std::vector<MetaFrame> metaFrame;

        std::list< std::pair<std::shared_ptr<TcameraData>, uint> > cam_callback_cache;

        unsigned char begin = 0;
        unsigned char end = 0; // one plus the end index
        bool imu_initialised;
        bool camera_initialised;

        std::vector<callbackSynced> _cblist_synced;
        void syncedCallback(const MetaFrame);
};

template <class TcameraData>
void CameraIMUSync<TcameraData>::push_backIMU(uint64_t monotonic_time, uint64_t timeSyncIn)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);

    if (!imu_initialised)
        imu_initialised = true;

    assert(MASK(begin) != MASK(end+1));

    // push back the data
    MetaFrame& frame = metaFrame[MASK(end)];
    
    frame.reset();
    frame.trigger_time = monotonic_time - timeSyncIn;
    end++;

    // std::cout << "IMU syncIn Time " <<  timeSyncIn << ", index " << MASK(end - 1) << std::endl;
    std::cout << "IMU " <<  monotonic_time << ", index " << MASK(end - 1) << std::endl;

    if(MASK(begin) == MASK(end+1)){ // remove the oldest record when full
        metaFrame[MASK(begin)].reset();
        begin++;
    }

    // Execute cached camera callbacks
    // the pop_size prevent circular calling
    auto pop_size = cam_callback_cache.size();
    while (pop_size--){
        auto param = cam_callback_cache.front();
        cam_callback_cache.pop_front();
        push_backCamera(param.first, param.second, false);
    }
}

template <class TcameraData>
void CameraIMUSync<TcameraData>::push_backCamera(std::shared_ptr<TcameraData> data, const uint index, bool master)
{
    std::scoped_lock lock(mtx, data->mtx);

    // the data might be uninitialised, if it is stored in the cam_callback_cache for a while
    if(!data->initialised()){
        return;
    }

    auto info = data->get_info();

    {  
        // debuging for OS processing latency
        struct timespec time_now;
        uint64_t monotonic_time;
        const int error = clock_gettime(CLOCK_MONOTONIC, &time_now);
        assert(error == 0);
        monotonic_time = (time_now.tv_sec * 1e9) + (time_now.tv_nsec);

        double delay_ms = (monotonic_time - info.capture_time_ns) / 1.0e6;

        std::cout << "push_backCamera(), index = " << index << ", OS delay " <<  delay_ms << "ms" << std::endl;
    }

    // if the camera is a master, add a fake imu measurement timing
    if(master)
        push_backIMU(info.capture_time_ns, 0);

    if (!camera_initialised){
        camera_initialised = true;

        // only keep the latest imu
        // if (imu_initialised)
        //     begin = MASK(end - 1);
    }

    if(!imu_initialised){
        data->release();
        return;
    }
    

    assert(index < Ncamera); // must not be out of range

    //// if it is to override
    // std::cout  << "index " << index << "begin = " << MASK(begin) << "end = " << MASK(end) << std::endl;
    assert(MASK(begin) != MASK(end+1));

    const uint64_t& camera_time = info.capture_time_ns - mean_delay;

    // start searching from the oldest imu readings, find the first imu reading that is within the max slack
    for (unsigned char i = MASK(begin); i != MASK(end); i = MASK(i+1))
    {
        MetaFrame& frame = metaFrame[i];
        // const uint64_t& imu_time = frame.imu.monotonic_time;
        // Use this to minus away the offset
        const uint64_t& imu_time = frame.trigger_time;

        

        // If camera is later than the imu time with maximum slack, skip
        if (imu_time + max_slack < camera_time)
                continue;
        
        // Reaching here, camera is within the slack time of the imu stamp

        // Conversion from unsigned to signed notation
        const double slack_sec = imu_time < camera_time ? 
            (camera_time - imu_time) / 1.0e9 : 
            - ((imu_time - camera_time) / 1.0e9) ;

        // Now, checking if the camera is not too far ahead the current imu (jitter)
        if ( imu_time - max_imu_read_jitter < camera_time){ // allow IMU to be slightly slower than camera, due to jitter
            
            std::cout << "Slack (ms) = " << slack_sec * 1e3 << ", Adding frame " << index << "( time " << camera_time << ") to IMU " << int(i) << "(" << imu_time << ")" << std::endl;
            const unsigned char bit = 1 << index;

            // Check if there is another frame already registered. If yes, then we encountered ambiguity in the sync, throw away the whole meta frame
            if((frame.cameraBitMask & bit) != 0){
                std::cout << "Detected ambiguity for camera " << index << "with capture time " << frame.camera[index]->get_info().capture_time_ns << std::endl;
                frame.cameraBitMask = frame.cameraBitMask & ~bit; // remove that active bit
                frame.reset();
                return;
            }

            frame.cameraBitMask = frame.cameraBitMask | bit;
            // TODO: currently, this only works if the sync is completed, before the next batch of frames comes. But this is normally the case
            // make initialised to false for all the rest of the data?
            frame.camera[index] = data;

            // detect if bitMask is full
            if (frame.isComplete(CAMERA_MASK))
            {

                if( i != MASK(begin)){
                    std::cerr << "[[[[[WARNING: Skipping " << MASK(i-begin) << " IMU data]]]]]" << std::endl;
                }       

                syncedCallback(frame);

                // reset all frames before this successful sync, as well as the current frame
                while (MASK(begin) != MASK(i+1)){
                    metaFrame[MASK(begin)].reset();
                    begin = MASK(begin+1);
                }

                std::cout << "IMU sync complete! for index " << MASK(i) << std::endl 
                    << "remaining data in buffer after success sync " << MASK(end - begin) << std::endl
                    << std::endl;
            }
            return;
        }else{
            std::cerr << "||||||||Failed to add camera frame: " << camera_time << std::endl;
            std::cerr << "Slack (ms) = " << slack_sec * 1e3 << std::endl; 

            data->release();
            return;
        }
        
    }

    // Reaching here, means the camera callback reaches earlier than imu callback

    std::cout << "Adding camera "<< index << " frame " << info.frame_count << " to cache..." << std::endl;
    assert(!master);
    cam_callback_cache.emplace_back(data, index);

    if (cam_callback_cache.size() > BUFFER_SIZE * Ncamera){
        std::cout << "camera " << index << " callback cache full, throwing frame " << cam_callback_cache.front().first->get_info().frame_count << std::endl;

        // releasing the memory of the shared_ptr
        cam_callback_cache.front().first->release();
        cam_callback_cache.pop_front();
    }

}

template <class TcameraData>
void CameraIMUSync<TcameraData>::syncedCallback(MetaFrame frame){

    for (auto& cb : _cblist_synced){
        cb(frame);
    }
}


#endif /* CAMERA_IMU_SYNC_HPP */
