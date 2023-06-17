#include <ros/ros.h>
#include <gst_pipeline.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>



int main(int arg, char *argv[]) {   

    ros::init(arg, argv, "image_publisher");    
    ros::NodeHandle node_handle; 
    image_transport::ImageTransport image_transporter(node_handle);
    image_transport::Publisher image_publisher = image_transporter.advertise("camera/image", 1);

    Pipeline pipeline(image_publisher);
   

    bool s = pipeline.init_pipeline();
    
    while (ros::ok())
    {
        pipeline.play_stream();

    }
    

    return 0;
}




