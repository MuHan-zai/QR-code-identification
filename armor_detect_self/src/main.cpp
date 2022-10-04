#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include "../include/armor_detect.h"
#include <cmath>
armor_detect::ArmorDetection armor_detection;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat img_;
    // std::cout << "111" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    cv_ptr->image.copyTo(img_);
    armor_detection.LoadImg(img_);
    armor_detection.LightsDetect();
    armor_detection.FindAllArmors();
    // armor_detection.DetectDir();
    armor_detection.DetectID();
    return;
}
int main(int argc,char **argv)
{
    // std::cout << "222" << std::endl;
    ros::init(argc,argv,"aaa");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    armor_detection.LoadParam(nh);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/mvsua_cam/image_raw1",1,imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}