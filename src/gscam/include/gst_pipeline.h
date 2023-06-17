#ifndef __gst_pipeline_H
#define __gst_pipeline_H

#include <iostream>
#include <image_transport/image_transport.h>
#include <gst/gst.h>
#include <torch/script.h> 



class Pipeline {

  private:
  const image_transport::Publisher m_publisher;

  GstElement *pipeline  = nullptr, *source  = nullptr, *videoconvert  = nullptr, *appsink = nullptr;
  GstStateChangeReturn ret;
  GstCaps* caps;
  gchar *debug_info;
  GstMessage *msg = nullptr;
  GstBuffer* buffer;
  GstMapInfo map;
  sensor_msgs::ImagePtr image_msg;
  GError *err = NULL;
  GstSample* sample;  
  GstPad* pad;
  int height, width, depth;
  torch::jit::script::Module module;

  public:
  Pipeline(const image_transport::Publisher& publisher);
  ~Pipeline();

  bool init_pipeline();
  void play_stream();

};

#endif
