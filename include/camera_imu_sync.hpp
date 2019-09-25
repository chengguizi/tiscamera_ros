/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-24 14:45:57 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2019-09-24 21:32:30
 */
#ifndef CAMERA_IMU_SYNC_HPP
#define CAMERA_IMU_SYNC_HPP

#include <cmath>

/**
 * Every IMU data coming in, will trigger a collection machanism, to collect incoming camera frames. 
 * The time stamps are strictly more recent, and no larger than the maximum duration
 * 
 * For simplicity and efficiency, here we assume IMU data always come in before all camera frames, in realtime
 */ 

template <class TimuData, class TcameraData, int Ncamera>
class CameraIMUSync{
    public:
        CameraIMUSync(){std::cout << "CAMERA_MASK = " << CAMERA_MASK << std::endl;};
        void set_camera_rate(double sec){max_duration = sec;};
        void push_backIMU(const TimuData& data);
        void push_backCamera(const TcameraData& data, const uint index);
        // bool try_sync();
    
    private:

        const static uint BUFFER_SIZE = 4;
        const static unsigned char BUFFER_MASK = BUFFER_SIZE - 1;
        #define MASK(x) ((x) & BUFFER_MASK)

        constexpr const static uint CAMERA_MASK = std::pow(2, Ncamera) - 1;

        double max_duration; // maximum duration an IMU will wait for a camera frame         
        std::mutex mtx;
        

        struct MetaFrame{
            TimuData imu;
            TcameraData camera[Ncamera];
            unsigned char cameraBitMask;

            void reset(){cameraBitMask = 0;};
        }metaFrame[BUFFER_SIZE];

        unsigned char begin = 0;
        unsigned char end = 0;
        unsigned char count = 0;
};

template <class TimuData, class TcameraData, int Ncamera>
void CameraIMUSync<TimuData,TcameraData, Ncamera>::push_backIMU(const TimuData& data)
{
    std::lock_guard<std::mutex> lock(mtx);

    // push back the data
    MetaFrame& frame = metaFrame[MASK(end)];
    
    frame.reset();
    frame.imu = data;
    end++; count++;

    std::cout << "IMU " <<  data.monotonic_time << ", index " << MASK(end - 1) << std::endl;

    if(count == BUFFER_SIZE){
        assert(MASK(begin) == MASK(end));
        // if (metaFrame[MASK(begin)].cameraBitMask != CAMERA_MASK) // not completed
        //     throw std::runtime_error("Overflow");

        std::cout << "Deleting index " << MASK(begin) << std::endl;
        
        begin++; count--;
    }
}

template <class TimuData, class TcameraData, int Ncamera>
void CameraIMUSync<TimuData,TcameraData, Ncamera>::push_backCamera(const TcameraData& data, const uint index)
{
    assert(index < Ncamera); // must not be out of range
    
    std::lock_guard<std::mutex> lock(mtx);

    const uint64_t& camera_time = data.capture_time_ns;

    // start searching from the beginning (oldest imu readings)
    for (unsigned char i = begin; MASK(i) != MASK(end); i++)
    {
        MetaFrame& frame = metaFrame[MASK(i)];
        const uint64_t& imu_time = frame.imu.monotonic_time;

        const double slack = (camera_time - imu_time) / 1.0e9;
        if ( imu_time < camera_time &&  slack < max_duration){
            std::cout << "Adding frame " << index << "( time " << camera_time << ") to IMU " << MASK(i) << "(" << imu_time << ")" << std::endl;
            std::cout << "Slack (ms) = " << slack * 1e3 << std::endl; 
            const unsigned char bit = 1 << index;
            assert( (frame.cameraBitMask & bit) == 0);

            frame.cameraBitMask = frame.cameraBitMask | bit;

            // detect if bitMask is full
            if (frame.cameraBitMask == CAMERA_MASK)
            {
                std::cout << "IMU sync complete! for index " << MASK(i) << std::endl;

                if(MASK(begin) != MASK(i)){
                    std::cerr << "Skipping " << MASK(i-begin) << " IMU data" << std::endl;
                }
                
                for (;MASK(begin) != MASK(i+1) ; begin++, count--)
                {
                    metaFrame[MASK(begin)].reset();
                }
                
            }

            return;
        }
    }
    
    std::cerr << "Failed to add frame " << index << std::endl;
}

// template <class TimuData, class TcameraData, int Ncamera>
// bool CameraIMUSync<TimuData,TcameraData, Ncamera>::try_sync()
// {
//     for (unsigned char i = 1; i <= BUFFER_SIZE - 1; i++)
//     {
//         const uint64_t& imu_time = imu[MASK(end - i)].monotonic_time;
//         const uint64_t& camera_time = camera.capture_time_ns;
//         if ( camera_time < imu_time) // skip all imu frames that are more recent than camera
//             continue;

//         camera.capture_time_ns = imu_time;

//         std::cout << "synced camera to imu time: " << imu_time << ", slack = " << (camera_time - imu_time) / 1.0e9 << std::endl;
//         return true;
//     }

//     return false;
// }


#endif /* CAMERA_IMU_SYNC_HPP */
