#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
extern "C"{
    #include <gst/app/gstappsink.h>
}
#include <gst_pipeline.h>
#include <utils.h>
#include <torch/script.h> 


Pipeline::Pipeline(const image_transport::Publisher& publisher) : m_publisher(publisher){
    height = 240, width = 320;
    //height = 480, width = 640;

    depth = 3;
    
}

Pipeline::~Pipeline(){}


bool Pipeline::init_pipeline(){

    if(!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    g_print( "Initializing gstreamer..." );
    gst_init(0,0);
    }
    // videostesrc for testing stream
    source = gst_element_factory_make("videotestsrc", "source");
    // v4l2src for webcam stream
    //source = gst_element_factory_make("v4l2src", "source");
    videoconvert = gst_element_factory_make("videoconvert", "video_convert");
    appsink = gst_element_factory_make("appsink", "app_sink");

    // building pipeline
    pipeline = gst_pipeline_new("pipeline");

    if (!pipeline || !source || !videoconvert || !appsink){
        g_print("Not all elements were instantiated!");
        return false;
    }

    gst_bin_add_many(GST_BIN (pipeline), source, videoconvert, appsink, NULL);
    if (gst_element_link_many(source, videoconvert, appsink, NULL) != TRUE){
        g_print("All elements could not be linked together");
        gst_object_unref(pipeline); 
        return false;
    }

    //caps = gst_caps_from_string("video/x-raw, format={BGR}, height={240}, width={320}");
    caps = gst_caps_new_simple ("video/x-raw",
               "format", G_TYPE_STRING, "RGB",
               "width", G_TYPE_INT, width,
               "height", G_TYPE_INT, height,
               "framerate", GST_TYPE_FRACTION, 30, 1,
               NULL);

    g_object_set (G_OBJECT (appsink), "emit-signals", true, "caps", caps, NULL);
    gst_caps_unref (caps);

    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE){
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return false;
    }
    
    g_printerr("Constructed the pipeline successfully.\n");

    module = load_model();

    return true;
}




void Pipeline::play_stream(){

    std::vector<torch::jit::IValue> inputs;
    inputs.reserve(1);

    sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));

    if(!sample){
        g_print("Could not pull gst sample!\n");
    }

    {
    buffer = gst_sample_get_buffer(sample);   
    gst_sample_unref (sample);     
    gst_buffer_map (buffer, &map, GST_MAP_READ);
    std::cout << "bufffer_size : " << map.size << "\n";

    at::Tensor tensor = torch::from_blob(map.data, {depth, height, width}).unsqueeze(0);
    std::cout << "input size: " << tensor.sizes() << std::endl;
    inputs.emplace_back(torch::from_blob(map.data, {depth, height, width}).unsqueeze(0));
    }

    at::Tensor output = module.forward(inputs).toTensor();
    std::cout << "output size: " << output.sizes() << std::endl;
    std::cout << "================================\n";


}


