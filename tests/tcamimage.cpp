#include "tcamimage.h"
#include <unistd.h>
#include <chrono>

#include <iostream>
#include <cassert>

#include "tiscamera_interface/tcam_statistics_meta.h"

using namespace gsttcam;

TcamImage::TcamImage(std::string serial) : TcamCamera(serial)
{
    _CustomData.ImageCounter = 0;
    _CustomData.SaveNextImage = false;
    _CustomData.bpp = 4;
    _CustomData.width = 0;
    _CustomData.height = 0;
    _CustomData.image_data = NULL;
}

TcamImage::~TcamImage()
{
    if(_CustomData.image_data != NULL)
    {
        delete _CustomData.image_data;
    }
}

void TcamImage::set_capture_format(std::string format, FrameSize size, FrameRate framerate)
{
    // Allocate memory for one image buffer.
    if( format == "GRAY8")
        _CustomData.bpp = 1;
    else
        _CustomData.bpp = 2;

    _CustomData.width = size.width;
    _CustomData.height = size.height;
    //_CustomData.image_data.resize( size.width * size.width * _CustomData.bpp );

    TcamCamera::set_capture_format(format, size, framerate);
}

bool TcamImage::start()
{
    // Register a callback to be called for each new frame
    set_new_frame_callback(new_frame_cb, &_CustomData);
    if(_CustomData.image_data != nullptr)
    {
        delete _CustomData.image_data;
    }
    _CustomData.image_data = new unsigned char[getImageDataSize()] ;
    TcamCamera::start();

    return true;
}

bool TcamImage::snapImage(int timeout_ms)
{
    std::unique_lock<std::mutex> lck(_CustomData.mtx);
    _CustomData.SaveNextImage = true;
    if( _CustomData.con.wait_for(lck,std::chrono::milliseconds(timeout_ms) ) != std::cv_status::timeout ){
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////
// Callback called for new images by the internal appsink
GstFlowReturn TcamImage::new_frame_cb(GstAppSink *appsink, gpointer data)
{
    // Cast gpointer to CUSTOMDATA*
    CUSTOMDATA *pCustomData = (CUSTOMDATA*)data;
    if( !pCustomData->SaveNextImage)
        return GST_FLOW_OK;
    std::lock_guard<std::mutex> lck(pCustomData->mtx);

    // pCustomData->SaveNextImage = false;
    pCustomData->ImageCounter++;

    // The following lines demonstrate, how to access the image
    // data in the GstSample.
    GstSample *sample = gst_app_sink_pull_sample(appsink);

    GstBuffer *buffer = gst_sample_get_buffer(sample);

    //// Obtain Metadata


    const GstMetaInfo * meta_info = gst_meta_get_info("TcamStatisticsMeta");

    if (meta_info != nullptr){
        TcamStatisticsMeta *meta = (TcamStatisticsMeta *) gst_buffer_get_meta(buffer, meta_info->api);

        // gint n_fields = gst_structure_n_fields(meta_tcam->structure);
        // std::cout << "n_fields = " << n_fields << std::endl;

        uint64_t frame_count;
        if (gst_structure_get_uint64(meta->structure, "frame_count", &frame_count))
            std::cout << "frame_count = " << frame_count << std::endl;

        uint64_t frames_dropped;
        if (gst_structure_get_uint64(meta->structure, "frames_dropped", &frames_dropped) && frames_dropped != 0)
            std::cout << "frames_dropped = " << frames_dropped << std::endl;
        
    }

        // gst_structure_set(struc,
        //               "frame_count", G_TYPE_UINT64, statistics->frame_count,
        //               "frames_dropped", G_TYPE_UINT64, statistics->frames_dropped,
        //               "capture_time_ns", G_TYPE_UINT64, statistics->capture_time_ns,
        //               "camera_time_ns", G_TYPE_UINT64, statistics->camera_time_ns,
        //               "framerate", G_TYPE_DOUBLE, statistics->framerate,
        //               "is_damaged", G_TYPE_BOOLEAN, statistics->is_damaged,
        //               nullptr);

    // FIRST TRY ///////////////////////////
    // gpointer state = nullptr;
    // GstMeta *meta;
    // const GstMetaInfo *meta_info = TCAM_STATISTICS_META_INFO;
    // while ((meta = gst_buffer_iterate_meta (buffer, &state))) {
    //     // if (meta->info->api == meta_info->api) { // matched!
    //     //     // TcamStatisticsMeta *tcam_meta = (TcamStatisticsMeta* )meta;

    //     //     // gint n_fields = gst_structure_n_fields(tcam_meta->structure);
    //     //     // std::cout << "n_fields = " << n_fields << std::endl;

    //     // }

    //      std::cout << "meta_count = "<< std::endl;

    // }

    /////////////////////////


    // SECOND TRY ////////////
    // TcamStatisticsMeta *meta;
    // meta = gst_buffer_get_tcam_statistics_meta(buffer);

    // if (meta != nullptr)
    // {
    //     gint n_fields = gst_structure_n_fields(meta->structure);
    //     std::cout << "n_fields = " << n_fields << std::endl;
    // }

    ///////////////////////////
    

    // std::cout << "meta_count = " << meta_count << std::endl;

    GstMapInfo info;

    gst_buffer_map(buffer, &info, GST_MAP_READ);
    
    if (info.data != NULL) 
    {
        // info.data contains the image data as blob of unsigned char 
        // memcpy( pCustomData->image_data, info.data, pCustomData->width * pCustomData->height * pCustomData->bpp);
        ////memcpy( pCustomData->image_data.data(), info.data, pCustomData->width * pCustomData->height * pCustomData->bpp);
    }


    
    // Calling Unref is important!
    gst_buffer_unmap (buffer, &info);
    gst_sample_unref(sample);

    pCustomData->con.notify_all();
    // Set our flag of new image to true, so our main thread knows about a new image.
    return GST_FLOW_OK;
}
