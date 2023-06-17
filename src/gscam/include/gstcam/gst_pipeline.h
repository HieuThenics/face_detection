#pragma oncce

class Student {

public:
    Student();
    void print();

};


#include <iostream>
#include <gst/gst.h>
#include<typeinfo>
#include<cxxabi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> 
#include <gst/gst.h>



class Pipeline {

  private:
  image_transport::Publisher m_publisher;

  GstElement *pipeline  = nullptr, *source  = nullptr, *videoconvert  = nullptr, *appsink = nullptr;
  GstBus *bus = nullptr;
  GstStateChangeReturn ret;
  GstCaps *caps;
  gchar *debug_info;
  GstMessage *msg = nullptr;
  GstBuffer* buffer;
  GstMapInfo map;
  sensor_msgs::ImagePtr image_msg;
  GError *err = NULL;
  GstSample* sample;

  public:
  Pipeline(image_transport::Publisher publisher);
  ~Pipeline();

  bool init_pipeline();
  void play_stream();

};

