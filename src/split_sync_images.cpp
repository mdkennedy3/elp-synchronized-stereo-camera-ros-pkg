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
    void SyncImageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sync_image_sub_;
    ros::Publisher left_image_pub_;
    ros::Publisher right_image_pub_;
    std::string camera_ns;
};


SplitImage::SplitImage()
{
  nh_ = ros::NodeHandle("~");
  sync_image_sub_  = nh_.subscribe("/elp/sync/image_raw", 1, &SplitImage::SyncImageCallback, this);
  nh_.param<std::string>("camera_ns", camera_ns, "elp"); //Set camera ns

  left_image_pub_ = nh_.advertise<sensor_msgs::Image>("left/image_raw",1);
  right_image_pub_ = nh_.advertise<sensor_msgs::Image>("right/image_raw",1);
}

void SplitImage::SyncImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //1. Given Image, convert to opencv mat  
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
     
    try
    {
      cv_ptr_left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_right = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int combined_rows = cv_ptr_left->image.rows; 
    int combined_cols = cv_ptr_left->image.cols;  
    int image_cols = combined_cols/2;
    int image_rows = combined_rows;
    //2. Given the width/height of the combined image, divide into two for the left/right images
    cv::Rect leftROI(0,0,image_cols,image_rows);
    cv::Rect rightROI(image_cols,0,image_cols,image_rows);
    cv::Mat leftcrop  = cv_ptr_left->image(leftROI);
    cv::Mat rightcrop  = cv_ptr_right->image(rightROI);
    cv_ptr_left->image = leftcrop;
    cv_ptr_right->image = rightcrop;
    //3. Publish the left and right images
    left_image_pub_.publish(cv_ptr_left->toImageMsg()); 
    right_image_pub_.publish(cv_ptr_right->toImageMsg());

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "split_sync_image_node");
  ROS_INFO("Split syncronized elp image node running");
  SplitImage splt_img_obj;
  ros::spin();
  return 0;
}


