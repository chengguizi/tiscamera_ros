/*
 * @Author: Cheng Huimin 
 * @Date: 2019-09-13 14:33:01 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2019-09-13 14:34:25
 */

#include "tiscamera_interface.hpp"

using namespace gsttcam;

TisCameraManager::TisCameraManager(std::string serial) : TcamCamera(serial)
{
    frame.data.frame_count = 0;
    frame.data.image_data = nullptr;
}

TisCameraManager::~TisCameraManager()
{

    // make sure all threads are shutdown, mutex released
    if(frame.data.image_data)
        delete frame.data.image_data;
}