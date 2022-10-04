#ifndef ARMOR_DETECT_H
#define ARMOR_DETECT_H
#include<opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
namespace armor_detect
{
    enum Mode
    {   
        Blue = 0,
        Red = 1,
        Lurker = 2
    };
    class Lights{
    public:
        Lights();
        Lights(std::vector<cv::Point> contour,cv::Mat src_img);
        ~Lights();
        cv::RotatedRect light_rect;
        cv::Point2f angular[2];// 灯条角点
        cv::Point2f armor_angular[2];// 用于armor初始化
        cv::Point2f center_;
        cv::Mat src_img_;
        std::vector<cv::Point> contour_;
        double angle;
        double light_aspect_ratio;// 长宽比
        double height_;
        double width_;
        double area_;
        int color;// 0为蓝色，1为红色

    private:
        void elementsCalc();//计算灯调角度，为长边与x轴夹角，范围为0-180,计算长宽比
        

    };
    class Armors{
        public:
            Armors();
            Armors(Lights &light1,Lights &light2,double &armor_angle_,double &lights_dis_,double &armor_height_max_);
            ~Armors();
            cv::Point2f points_[4];
            cv::RotatedRect  num_rect;
            cv::Point2f center_;
            int color;
            double height;
            double width;
            double angle;
            Lights lights_[2];
        private:
            

    };
    class ArmorDetection{
    public:
        ArmorDetection();
        void LoadImg(cv::Mat &img_);
        void LoadParam(ros::NodeHandle& nh);
        void LightsDetect();
        void FindAllArmors();
        void DetectID();
        // void DetectDir();
        ~ArmorDetection();
    private:
        std::vector<Lights> lights_;
        std::vector<Armors> armors_;
        std::vector<Armors> armors_removed;
        cv::Mat img,img_original;
        int binary_threshold;//原始图片二值化阈值
        Lights light_;
        Mode mode_;
        double light_ratio_min;
        double light_ratio_max;
        double light_max_angle_;
        double light_min_angle_;
        double armor_max_angle_;
        double light_max_angle_diff_;
        double armor_max_ratio_;
        double armor_min_ratio_;
        double armor_light_ratio_;
        static int ttt;
        bool debug_flag;
        int dir ;
        
    };
    
}

#endif