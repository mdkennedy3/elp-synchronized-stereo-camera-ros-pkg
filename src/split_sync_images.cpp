/*
Author: Monroe Kennedy III
Description:
This node simply takes in the syncronized images from the ELP camera, 
provided the width of the images splits the image into messages for left/right camera (still synchronized).
*/

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>

class SplitImage
{
  public:
    SplitImage(); 
    void SyncImageCallback(const sensor_msgs::Image& msg);
    void Publish_image(std::string camera); 
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sync_image_sub_;
    std::string camera_ns;
};


SplitImage::SplitImage()
{
  nh_ = ros::NodeHandle("~");
  sync_image_sub_  = nh_.subscribe("", 1, &SplitImage::SyncImageCallback, this);
  nh_.param<std::string>("camera_ns", camera_ns, "elp"); //Set camera ns

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "split_sync_image_node");
  ROS_INFO("Split syncronized elp image node running");
  SplitImage splt_img_obj;
  ros::spin();
  return 0;
}


