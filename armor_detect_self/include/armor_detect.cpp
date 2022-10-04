#include "armor_detect.h"

namespace armor_detect
{
// +########################################################### ArmorDetection #######################################################################################
    ArmorDetection::ArmorDetection(){}
    void ArmorDetection::LoadImg(cv::Mat& img_)
    {
        lights_.clear();
        armors_.clear();
        armors_removed.clear();
        img = img_;
        img_original = img_.clone();
    }
    void ArmorDetection::LoadParam(ros::NodeHandle& nh)
    {
        int mode_m;
        nh.param("/armor_detect_node/ArmorDetect/binary_threshold", binary_threshold, 110);
        nh.param("/armor_detect_node/ArmorDetect/mode", mode_m, 0);
        nh.param("/armor_detect_node/ArmorDetect/light_ratio_max", light_ratio_max, 8.0);
        nh.param("/armor_detect_node/ArmorDetect/light_ratio_min", light_ratio_min, 1.5);
        nh.param("/armor_detect_node/ArmorDetect/light_max_angle_", light_max_angle_, 120.0);
        nh.param("/armor_detect_node/ArmorDetect/light_min_angle_", light_min_angle_, 60.0);
        nh.param("/armor_detect_node/ArmorDetect/armor_max_angle_", armor_max_angle_, 30.0);
        nh.param("/armor_detect_node/ArmorDetect/light_max_angle_diff_", light_max_angle_diff_, 15.0);
        nh.param("/armor_detect_node/ArmorDetect/armor_max_ratio_", armor_max_ratio_, 3.8);
        nh.param("/armor_detect_node/ArmorDetect/armor_min_ratio_", armor_min_ratio_, 0.8);
        nh.param("/armor_detect_node/ArmorDetect/armor_light_ratio_", armor_light_ratio_, 1.5);
        nh.param("/armor_detect_node/ArmorDetect/debug_flag", debug_flag, false);
        if(mode_m == 0)mode_=Blue;
        if(mode_m == 1)mode_=Red;
        if(mode_m == 2)mode_=Lurker;
    }
    void ArmorDetection::LightsDetect()
    {
        // 二值化图片
        cv::Mat img_copy;
        cv::Mat gray_img;
        cv::Mat binary_img;
        cv::cvtColor(img,gray_img,cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img,binary_img,binary_threshold,255,cv::THRESH_BINARY);
        img_copy = img.clone();
        // 寻找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(img_copy,contours,-1,cv::Scalar(0,255,0),2);
        cv::imshow("img_copy",img_copy);
        // 开始筛选灯条
        for(int i = 0;i < contours.size(); i++)
        {
            light_ =  Lights(contours[i],img);
            cv::Point2f vertex_lights[4];
            light_.light_rect.points(vertex_lights);
            for (int i = 0; i < 4; i++)
            {
                cv::line(img_copy, vertex_lights[i], vertex_lights[(i + 1) % 4], cv::Scalar(255, 100, 200),2,CV_AA);
            }
            cv::imshow("img_copy1",img_copy);
            // 面积过小的排除
            if(light_.area_ < 6)continue;
            // 长宽比排除
            if(light_.light_aspect_ratio < light_ratio_min ||light_.light_aspect_ratio > light_ratio_max)continue;
            // 角度排除
            if(light_.angle > light_max_angle_ || light_.angle < light_min_angle_)continue;
            // 颜色判断
            if((mode_ == Blue || mode_ == Lurker)&&light_.color == 0)
            {
                lights_.emplace_back(light_);
            }
            if((mode_ == Red || mode_ == Lurker)&&light_.color == 1)
            {
                lights_.emplace_back(light_);
            }
        }
    }
    void ArmorDetection::FindAllArmors()
    {
        if(lights_.size() < 2)return;
        // 灯条两两匹配
        for(int i = 0; i < lights_.size(); i++)
        {
            for(int j = i+1; j < lights_.size(); j++)
            {
                Lights light1 = lights_[i];
                Lights light2 = lights_[j];
                double lights_dis = std::sqrt(std::pow(light1.center_.x-light2.center_.x, 2) + std::pow(light1.center_.y-light2.center_.y, 2));
                double armor_height_max = std::max<double>(light1.height_, light2.height_);
                double armor_height_min = std::min<double>(light1.height_, light2.height_);
                double angle_diff = std::fabs(light1.angle - light2.angle);
                 double armor_angle = 57.2957795 * std::atan2(light1.center_.y-light2.center_.y, light1.center_.x-light2.center_.x);
                // 颜色不一致排除 yipaichu
                if(light1.color != light2.color)
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                }
                // 灯条中心相对位置排除 yipaichu
                if(fabs(light1.center_.x - light2.center_.x) < 10 && fabs(light1.center_.y - light2.center_.y) > 35) 
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                }
                // 灯条角度差排除 yipaichu
                if(angle_diff > light_max_angle_diff_) 
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                }
                // 装甲角度排除 yipaichu
                if(std::fabs(armor_angle) > 90) armor_angle += 180 * (armor_angle>0 ? -1 : 1);
                if(std::fabs(armor_angle) > armor_max_angle_) 
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                } 
                // 装甲长宽比排除
                if(std::fabs(armor_height_min) < 1e-3 || std::fabs(armor_height_max) < 1e-3) 
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                }//排除装甲高度太大或太小的 yipaichu
                if(lights_dis / armor_height_max > armor_max_ratio_) 
                {
                    // armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                } //排除装甲长宽比大
                if(lights_dis / armor_height_max < armor_min_ratio_) 
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                } //排除装甲长宽比小
                if(armor_height_max / armor_height_min > armor_light_ratio_)
                {
                    armors_removed.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
                    continue;
                } //排除装甲灯条比例相差大 yipaichu
                armors_.emplace_back(Armors(light1,light2,armor_angle,lights_dis,armor_height_max));
            }
        }
        
    }
     void ArmorDetection::DetectID()
    {
       
        double nub_angle;
        double angle;
        int x, y, height, width;

        cv::Mat rotation, img_wrap, num, num_part;
        cv::threshold(img_wrap, img_wrap, 255, 255, cv::THRESH_BINARY);
        std::vector<std::vector<cv::Point>> contours3;
        std::vector<cv::Vec4i> hierarchy3;
        cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::Point2f vertex[4];
        cv::Point2f vertex_changed[4];
        cv::Rect rect;
        int k, final_contour_index;
        double max_contour = 0.0, temp_contour;
        double area_final;
        rect = cv::Rect(20,115,160, 75);
        vertex_changed[0] = cv::Point2f(0, 240);
        vertex_changed[1] = cv::Point2f(0, 0);
        vertex_changed[2] = cv::Point2f(120, 0);
        vertex_changed[3] = cv::Point2f(120, 240);
// #####################################################################
        int x_,y_;
        static int ttt1_= 0;
        cv::Mat img_gray_,rotation_,img_gray1,img_gray_dir,img_gray_id;
        // 透视变换
        cv::Point2f vertex_[4];
        cv::Point2f vertex_changed_[4];
        vertex_changed_[0] = cv::Point2f(0,300);// 变换后坐标
        vertex_changed_[1] = cv::Point2f(0,0);
        vertex_changed_[2] = cv::Point2f(300,0);
        vertex_changed_[3] = cv::Point2f(300,300);
        for (unsigned int i = 0; i < armors_.size(); i++)
        {
            if(!debug_flag)
            {
                char rout[100];
                double con_angle;
                double con_width;
                double con_height_;
                int pix1 = 0;
                int pix2 = 0;
                int pix3 = 0;
                int pix4 = 0; 
                bool p1_order = false;
                bool p2_order = false;
                bool p3_order = false;
                bool p4_order = false;
                std::vector<std::vector<cv::Point>> contours3_;
                std::vector<cv::Vec4i> hierarchy3_;
                cv::Mat element1_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
                cv::Mat element2_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                vertex_[0] =  armors_[i].points_[0];// 变换前坐标
                vertex_[1] =  armors_[i].points_[1];
                vertex_[2] =  armors_[i].points_[2];
                vertex_[3] =  armors_[i].points_[3];
                x_ = armors_[i].center_.x;
                y_ = armors_[i].center_.y;
                rotation_ = cv::getPerspectiveTransform(vertex_,vertex_changed_);
                cv::warpPerspective(img_original,img_gray_,rotation_,cv::Size(300,300));
                cv::cvtColor(img_gray_, img_gray_, cv::COLOR_BGR2GRAY);
                cv::Rect rect_(20,0,260,300);
                cv::Rect rect1_(0,0,70,90);//左上
                cv::Rect rect2_(190,0,70,90);//右上
                cv::Rect rect3_(0,210,70,90);//左下
                cv::Rect rect4_(190,210,70,90);//左上
                cv::Mat block1;
                cv::Mat block2;
                cv::Mat block3;
                cv::Mat block4;
                img_gray_ = img_gray_(rect_);
                // cv::imshow("img_src", img_gray_);
                cv::equalizeHist(img_gray_,img_gray_);
                // cv::imshow("img_Hist", img_gray_);
                // cv::medianBlur(img_gray_,img_gray_,3);
                // cv::imshow("img_Blur", img_gray_);
                cv::adaptiveThreshold(img_gray_,img_gray_dir,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,301,-2);
                cv::erode(img_gray_dir, img_gray_dir, element1);
                // cv::erode(img_gray_, img_gray_, element1);
                cv::dilate(img_gray_dir, img_gray_dir, element2_);
                cv::dilate(img_gray_dir, img_gray_dir, element1);
                // cv::dilate(img_gray_, img_gray_, element1);
                // cv::dilate(img_gray_, img_gray_, element1);
                // cv::dilate(img_gray_, img_gray_, element1);
                // cv::imshow("img_gray_", img_gray_);
                block1 = img_gray_dir(rect1_);
                block2 = img_gray_dir(rect2_);
                block3 = img_gray_dir(rect3_);
                block4 = img_gray_dir(rect4_);
                // sprintf(rout,"/home/hanhan/catkin_ws/src/armor_detect_self/photo/%d.jpg",ttt1_++);
                // cv::imwrite(rout,img_gray_);

                // 计算各块区域中像素值总和
                for(int i = 0; i < block1.rows; i++)
                {
                    uchar* data1 = block1.ptr<uchar>(i);
                    uchar* data2 = block2.ptr<uchar>(i);
                    uchar* data3 = block3.ptr<uchar>(i);
                    uchar* data4 = block4.ptr<uchar>(i);
                    for(int j = 0; j < block1.cols; j++)
                    {
                        pix1 += data1[j];
                        pix2 += data2[j];
                        pix3 += data3[j];
                        pix4 += data4[j];
                    }
                }
                // 四个数排序
                if(pix3 > std::max<int>(pix1, pix2) || pix3 > std::max<int>(pix1, pix4) || pix3 > std::max<int>(pix4, pix2))p3_order = true;
                if(pix1 > std::max<int>(pix3, pix2) || pix1 > std::max<int>(pix2, pix4) || pix1 > std::max<int>(pix4, pix3))p1_order = true;
                if(pix4 > std::max<int>(pix1, pix2) || pix4 > std::max<int>(pix1, pix3) || pix4 > std::max<int>(pix3, pix2))p4_order = true;
                if(pix2 > std::max<int>(pix1, pix3) || pix2 > std::max<int>(pix1, pix4) || pix2 > std::max<int>(pix4, pix3))p2_order = true;
                if(p1_order and p2_order)dir = 2;//B
                if(p3_order and p4_order)dir = 0;//F
                if(p1_order and p3_order)dir = 3;//R
                if(p2_order and p4_order)dir = 1;//L

                // cv::equalizeHist(img_gray_,img_gray_);
                // cv::medianBlur(img_gray_,img_gray_,3);
                // // cv::imshow("img_src_", img_gray_);
                // cv::adaptiveThreshold(img_gray_,img_gray_,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,301,-2);
                // cv::Rect rect1_(20,75,40,100);
                // cv::imshow("img_gray_", img_gray_);
                // sprintf(rout,"/home/hanhan/catkin_ws/src/armor_detect_self/photo/%d.jpg",ttt++);
                // cv::imwrite(rout,img_gray_);
                // img_gray_ = img_gray_(rect1_);
                // cv::threshold(img_gray_,img_gray_,125,255,cv::THRESH_BINARY_INV);
                // // sprintf(rout,"/home/hanhan/catkin_ws/src/armor_detect_self/photo/%d.jpg",ttt++);
                // // cv::imwrite(rout,img_gray_);
                // cv::findContours(img_gray_, contours3_, hierarchy3_, cv::RETR_EXTERNAL, cv::RETR_CCOMP);
                // for (int k = 0; k < contours3_.size(); k++)
                //     {
                //         cv::RotatedRect rect1_ = cv::minAreaRect(contours3_[k]);
                //         if(rect1_.size.width < rect1_.size.height) con_angle = -(rect1_.angle - 90);
                //         else con_angle = -rect1_.angle;
                //         con_height_ = std::max(rect1_.size.width, rect1_.size.height);
                //         con_width = std::min(rect1_.size.width, rect1_.size.height);
                //         if(con_height_ > 30)std::cout << con_angle << std::endl;
                //         if(con_angle > 88 and con_angle < 92 and con_height_ > 40 and con_width < 14)
                //         {
                //             dir = 3;// 右
                //             break;
                //         }
                //         if(con_height_ > 30 and con_angle > 50 and con_angle < 83)
                //         {
                //             dir = 0;//前
                //             break;
                //         }
                //         if(con_height_ > 30 and con_angle > 110 and con_angle < 160)
                //         {
                //             dir = 2;//后
                //             break;
                //         }
                //     }
              //         cv::imshow("img_gray_", img_gray_);
                // ####################################################################
                char Dir[3];
                // armors_[i].num_rect.points(vertex);
                x = armors_[i].center_.x;
                y = armors_[i].center_.y;
                // rotation = cv::getPerspectiveTransform(vertex, vertex_changed);
                // cv::warpPerspective(img, img_wrap, rotation, cv::Size(120, 240));
                cv::Rect rect__(50,10,200,280);
                cv::adaptiveThreshold(img_gray_,num,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,301,-80);
                // cv::erode(img_wrap, num, element1);
                
                cv::imshow("num", num);
                num = num(rect__);
                num_part = num(rect);
                // cv::erode(num_part, num_part, element1);
                cv::dilate(num_part, num_part, element1);
                cv::dilate(num_part, num_part, element1);
                cv::dilate(num_part, num_part, element1);
                cv::imshow("num_part", num_part);
                cv::findContours(num_part, contours3, hierarchy3, cv::RETR_EXTERNAL, cv::RETR_CCOMP);
                if (!contours3.empty())
                {
                    max_contour = 0;
                    for (k = 0; k < contours3.size(); k++)
                    {
                        temp_contour = cv::contourArea(contours3[k]);
                        if (temp_contour > max_contour)
                        {
                            max_contour = temp_contour;
                            final_contour_index = k;
                        }
                    }
                    // std::cout << "max_area" << max_contour << std::endl;

                    // if (max_contour > 300 and max_contour < 500)
                    // {
                    //     //  cv::imwrite("./400.png",num_part);
                    //     cv::dilate(num_part, num_part, element1);
                    //     cv::dilate(num_part, num_part, element1);
                    //     cv::dilate(num_part, num_part, element1);
                    //     cv::findContours(num_part, contours3, hierarchy3, cv::RETR_EXTERNAL, cv::RETR_CCOMP);
                    //     if (!contours3.empty())
                    //     {
                    //         max_contour = 0;
                    //         for (k = 0; k < contours3.size(); k++)
                    //         {
                    //             temp_contour = cv::contourArea(contours3[k]);
                    //             if (temp_contour > max_contour)
                    //             {
                    //                 max_contour = temp_contour;
                    //                 final_contour_index = k;
                    //             }
                    //         }
                    //     }
                    // }

                    // if (max_contour > 300)
                    // {
                        cv::RotatedRect rect1 = cv::minAreaRect(contours3[final_contour_index]);
                        if (rect1.size.width < rect1.size.height)
                        { //最后得到的角度均为X轴与长边的夹角且为负值
                            angle = rect1.angle - 90;
                        }
                        else
                        {
                            angle = rect1.angle;
                        }
                        angle *= -1;
                        // std::cout<< angle <<std::endl;
                        if(dir == 0)sprintf(Dir,"%C",'F');
                        if(dir == 1)sprintf(Dir,"%C",'L');
                        if(dir == 2)sprintf(Dir,"%C",'B');
                        if(dir == 3)sprintf(Dir,"%C",'R');
                        if (angle > 75)
                        {

                            cv::putText(img, "1 ", cv::Point(x, y + 120), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4, 8);
                            cv::putText(img, Dir, cv::Point(x - 40, y + 120), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 4, 8);
                        }
                        else if (angle < 75)
                        {
                            cv::putText(img, "2", cv::Point(x, y + 120), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4, 8);
                            cv::putText(img, Dir, cv::Point(x - 40, y + 120), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 4, 8);
                        }
                    // }
                    // cv::imshow("img_wrap", img_wrap);
                    // cv::imshow("num", num);
                    // cv::imshow("num_part", num_part);

                    for (int j = 0; j < 4; j++)
                        cv::line(img, armors_[i].points_[j], armors_[i].points_[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
                else
                {
                    std::cout << "no contours" << std::endl;
                    for (int j = 0; j < 4; j++)
                        cv::line(img, armors_[i].points_[j], armors_[i].points_[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
            }
            else
                {
                    // std::cout << "no contours" << std::endl;
                    for (int j = 0; j < 4; j++)
                        cv::line(img, armors_[i].points_[j], armors_[i].points_[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
        }
        cv::imshow("view", img);
        // cv::waitKey(30);
        return;
    }
    // void ArmorDetection::DetectDir()
    // {
    //     int x_,y_;
    //     static int ttt1_= 0;
    //     cv::Mat img_gray_,rotation_,img_gray1;
    //     // 透视变换
    //     cv::Point2f vertex_[4];
    //     cv::Point2f vertex_changed_[4];
    //     vertex_changed_[0] = cv::Point2f(0,300);// 变换后坐标
    //     vertex_changed_[1] = cv::Point2f(0,0);
    //     vertex_changed_[2] = cv::Point2f(300,0);
    //     vertex_changed_[3] = cv::Point2f(300,300);
    //     for(unsigned int i = 0;i < armors_.size();i++)
    //     {
    //         char rout[100];
    //         double con_angle;
    //         double con_width;
    //         double con_height_;
    //         std::vector<std::vector<cv::Point>> contours3_;
    //         std::vector<cv::Vec4i> hierarchy3_;
    //         cv::Mat element1_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    //         cv::Mat element2_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    //         vertex_[0] =  armors_[i].points_[0];// 变换前坐标
    //         vertex_[1] =  armors_[i].points_[1];
    //         vertex_[2] =  armors_[i].points_[2];
    //         vertex_[3] =  armors_[i].points_[3];
    //         x_ = armors_[i].center_.x;
    //         y_ = armors_[i].center_.y;
    //         rotation_ = cv::getPerspectiveTransform(vertex_,vertex_changed_);
    //         cv::warpPerspective(img_original,img_gray_,rotation_,cv::Size(300,300));
    //         cv::cvtColor(img_gray_, img_gray_, cv::COLOR_BGR2GRAY);
    //         cv::Rect rect_(20,0,260,300);
    //         cv::Rect rect1_(20,75,40,100);
    //         img_gray_ = img_gray_(rect_);
    //         cv::equalizeHist(img_gray_,img_gray_);
    //         cv::medianBlur(img_gray_,img_gray_,3);
    //         cv::imshow("img_src_", img_gray_);
    //         cv::adaptiveThreshold(img_gray_,img_gray_,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,31,0);
    //         img_gray_ = img_gray_(rect1_);
    //         cv::threshold(img_gray_,img_gray_,125,255,cv::THRESH_BINARY_INV);
    //         // sprintf(rout,"/home/hanhan/catkin_ws/src/armor_detect_self/photo/%d.jpg",ttt++);
    //         // cv::imwrite(rout,img_gray_);
    //         cv::findContours(img_gray_, contours3_, hierarchy3_, cv::RETR_EXTERNAL, cv::RETR_CCOMP);
    //         for (int k = 0; k < contours3_.size(); k++)
    //             {
    //                 cv::RotatedRect rect1_ = cv::minAreaRect(contours3_[k]);
    //                 if(rect1_.size.width < rect1_.size.height) con_angle = -(rect1_.angle - 90);
    //                 else con_angle = -rect1_.angle;
    //                 con_height_ = std::max(rect1_.size.width, rect1_.size.height);
    //                 con_width = std::min(rect1_.size.width, rect1_.size.height);
    //                 if(con_height_ > 30)std::cout << con_angle << std::endl;
    //                 if(con_angle > 88 and con_angle < 92 and con_height_ > 40 and con_width < 14)
    //                 {
    //                     dir = 3;// 右
    //                     break;
    //                 }
    //                 if(con_height_ > 30 and con_angle > 50 and con_angle < 83)
    //                 {
    //                     dir = 0;//前
    //                     break;
    //                 }
    //                 if(con_height_ > 30 and con_angle > 110 and con_angle < 160)
    //                 {
    //                     dir = 2;//后
    //                     break;
    //                 }
    //             }
    //         cv::imshow("img_gray_", img_gray_);

    //     }
        
        
    // }
    ArmorDetection::~ArmorDetection(){}
// +########################################################### Lights #######################################################################################
    Lights::Lights(){}
    Lights::Lights(std::vector<cv::Point> contour,cv::Mat src_img)
    {
        contour_ = contour;
        light_rect = cv::minAreaRect(contour);
        src_img_ = src_img;
        elementsCalc();
    }
    void Lights::elementsCalc()
    {
        // 角度计算
        if(light_rect.size.width < light_rect.size.height) angle = -(light_rect.angle - 90);
        else angle = -light_rect.angle;
        // 长，宽 长宽比计算
        height_ = std::max(light_rect.size.width, light_rect.size.height);
        width_ = std::min(light_rect.size.width, light_rect.size.height);
        light_aspect_ratio = height_/ width_;
        // 中心点坐标
        center_.x = light_rect.center.x;
        center_.y = light_rect.center.y;
        // 面积计算
        area_ = light_rect.size.area();
        // 两端点坐标计算
        double rad_angle = angle/57.2957795;// 转弧度制
        double height_half = height_/2;
        angular[0] = (cv::Point2f(center_.x-height_half*cos(rad_angle),center_.y+height_half*sin(rad_angle)));//下点
        angular[1] = (cv::Point2f(center_.x+height_half*cos(rad_angle), center_.y-height_half*sin(rad_angle)));//上点
        armor_angular[0] = (cv::Point2f(center_.x-2.212*height_half*cos(rad_angle),center_.y+2.212*height_half*sin(rad_angle)));//下点
        armor_angular[1] = (cv::Point2f(center_.x+2.212*height_half*cos(rad_angle), center_.y-2.212*height_half*sin(rad_angle)));//上点
        // 颜色判断
        cv::Rect rect = cv::boundingRect(contour_);
        cv::Mat rect_color = src_img_(rect);
        cv::Mat color_split[3],rect_red,rect_blue;
        cv::split(rect_color,color_split);
        cv::subtract(color_split[0], color_split[2], rect_blue);
        cv::subtract(color_split[2], color_split[0], rect_red);
        int blue_pixel_sum = 0, red_pixel_sum = 0;
        for(int i = 0; i < rect_blue.rows; i++){
            uchar* data_blue = rect_blue.ptr<uchar>(i);
            uchar* data_red = rect_red.ptr<uchar>(i);
            for(int j = 0; j < rect_blue.cols; j++){
                blue_pixel_sum += data_blue[j];
                red_pixel_sum += data_red[j];
            }
        }
        if(blue_pixel_sum > red_pixel_sum * 2 )
        {
            color = 0;//blue
        }
        else
        {
            color = 1;//red
        }
    }
    
    Lights::~Lights(){}
// +########################################################### Armors #######################################################################################    
    Armors::Armors(){}
    Armors::Armors(Lights &light1,Lights &light2,double &armor_angle_,double &lights_dis_,double &armor_height_max_)
    {
        angle = armor_angle_;
        color = light1.color;
        center_.x = (light1.center_.x + light2.center_.x) / 2;
        center_.y = (light1.center_.y + light2.center_.y) / 2;
        width = lights_dis_;
        height = armor_height_max_;
        if(light1.center_.x < light2.center_.x)
        {
            lights_[0] = light1;
            lights_[1] = light2;
            points_[0] = light1.armor_angular[0];
            points_[1] = light1.armor_angular[1];
            points_[3] = light2.armor_angular[0];
            points_[2] = light2.armor_angular[1];
            
        }
        else
        {
            lights_[1] = light1;
            lights_[0] = light2;
            points_[3] = light1.armor_angular[0];
            points_[2] = light1.armor_angular[1];
            points_[0] = light2.armor_angular[0];
            points_[1] = light2.armor_angular[1];
            
        }
        num_rect.center.x = (light1.center_.x + light2.center_.x) / 2;
        num_rect.center.y = (light1.center_.y + light2.center_.y) / 2;
        num_rect.size.width = lights_dis_ * 0.6;
        num_rect.size.height = armor_height_max_ * 2;
    }   
    Armors::~Armors(){}
}