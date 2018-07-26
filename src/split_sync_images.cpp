/*
Author: Monroe Kennedy III
Description:
This node simply takes in the syncronized images from the ELP camera, 
provided the width of the images splits the image into messages for left/right camera (still synchronized).
*/
#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>

class SplitImage
{
  public:
    SplitImage(); 
    void SyncImageCallback(const sensor_msgs::ImageConstPtr& msg);
  private:
    ros::NodeHandle nh_;
    /** image transport interfaces */
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sync_image_sub_;    
    std::shared_ptr<image_transport::CameraPublisher> left_image_pub_;
    std::shared_ptr<image_transport::CameraPublisher> right_image_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;
    std::string camera_ns; //Camera ns
    //Path to calibration files
    std::string calib_path_;
};

SplitImage::SplitImage() //: nh_(ros::NodeHandle("~")), it_(std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_))), left_image_pub_(it_->advertise("left/image_raw",1)), right_image_pub_ (it_->advertise("right/image_raw",1))
{
  nh_ = ros::NodeHandle("~");
  it_.reset(new image_transport::ImageTransport(nh_)); //connect image transport to node
  cinfo_left_= std::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_, "elp_left"));  //Setup camera info managers in respective ns
  cinfo_right_= std::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_, "elp_right"));  
  //Load calibration files
  cinfo_left_->loadCameraInfo("package://elp_stereo_synchronized_ros_pkg/calibration/elp_left.yaml");
  cinfo_right_->loadCameraInfo("package://elp_stereo_synchronized_ros_pkg/calibration/elp_right.yaml");
  //sync_image_sub_  = nh_.subscribe("/elp/sync/image_raw", 1, &SplitImage::SyncImageCallback, this);
  nh_.param<std::string>("camera_ns", camera_ns, "elp"); //Set camera ns
  left_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_->advertiseCamera("left/image_raw",1));
  right_image_pub_ = std::make_shared<image_transport::CameraPublisher>(it_->advertiseCamera("right/image_raw",1));
  sync_image_sub_  = it_->subscribe("/"+camera_ns+"/sync/image_raw", 1, &SplitImage::SyncImageCallback, this);
}

void SplitImage::SyncImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //1. Given Image, convert to opencv mat  
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    std::string left_frame = camera_ns + "_left_optical_frame";
    std::string right_frame = camera_ns + "_right_optical_frame";
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
    cv_ptr_left->header.frame_id = left_frame;
    cv_ptr_right->header.frame_id = right_frame;
    //Get camera infos
    sensor_msgs::CameraInfoPtr ci_left_(new sensor_msgs::CameraInfo(cinfo_left_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr ci_right_(new sensor_msgs::CameraInfo(cinfo_right_->getCameraInfo()));
    ci_left_->header = cv_ptr_left->header;
    ci_right_->header = cv_ptr_right->header;
    //3. Publish the left and right images
    left_image_pub_->publish(cv_ptr_left->toImageMsg(),ci_left_); 
   right_image_pub_->publish(cv_ptr_right->toImageMsg(),ci_right_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "split_sync_image_node");
  ROS_INFO("Split syncronized elp image node running");
  SplitImage splt_img_obj;
  ros::spin();
  return 0;
}
