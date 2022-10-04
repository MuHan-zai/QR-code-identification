#include<opencv2/opencv.hpp>
#include"../include/tag_track.h"
#include<ros/ros.h>

tag_track:: TagTrack tag_track_;
int main(int argc,char **argv)
{
    ros::init(argc,argv,"hhh");
    ros::NodeHandle h;
    cv::Mat frame;
    cv::VideoCapture cap;
    cv::namedWindow("frame");
    ros::Rate loop_rate(50);
    cv::startWindowThread();
    if(!cap.open(0))
        return -1;
    while(ros::ok())
    {
        cap >> frame;
        ros::spinOnce();
        loop_rate.sleep();
        tag_track_.TagIdentify(frame);
       
    }
    cv::destroyWindow("frame");
    
    return 0;
}