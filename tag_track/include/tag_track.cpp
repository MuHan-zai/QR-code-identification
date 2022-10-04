#include "tag_track.h"
#include<cmath>
namespace tag_track
{
    Tag::Tag() 
    {
        
        
    }
    Tag::Tag(cv::Mat &tag,cv::Rect &rect_)
    {
        tag_region = tag;
        rect = rect_;
        // NUM[0] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,0,0,1,1,1,0,0};
        // NUM[1] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,0,1,1,0,0,1,0,1,1,1};
        // NUM[2] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1};
        // NUM[3] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,0,1,0,0,0,1,1,1,1};
        // NUM[4] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,0,1,1,0,1,0,1,1,0,1};
        // NUM[5] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,0,1,0,0,1,1,0};
        // NUM[6] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,0,0,1,0,0,0,0,1,0,0};
        // NUM[7] = {0,0,0,1,0,1,1,0,0,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1};
        // NUM[8] = {0,0,0,1,0,1,1,0,0,1,1,0,1,0,1,0,0,1,1,0,1,1,1,0,1};
    
    }
    Tag::~Tag(){}
    
    TagTrack::TagTrack() 
    {
        
    }
    void TagTrack::TagIdentify(cv::Mat &frame)
    {
        //读取图像并将其二值化
        cv::Mat gray_img, binary_img;
        cv::Mat img_cp;
        // src_img = cv::imread("src/tag_track/photo/tag.jpg");
        src_img = frame;
        img_cp = src_img.clone();
        // std::cout << src_img.size() << std::endl;
        cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
        cv::imshow("gray_img",gray_img);
        cv::blur(gray_img, gray_img,cv::Size(3,3));
        cv::Canny(gray_img, binary_img,220,150,3);
        cv::imshow("binary_img",binary_img);
        // // std::cout << gray_img.size() << std::endl;
        // cv::adaptiveThreshold(gray_img,binary_img,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,1281,20);        // 寻找轮廓
        // cv::imshow("binary_img",binary_img);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<std::vector<cv::Point>> contours_approx;
        std::vector<std::vector<cv::Point>> contours_tag;
        std::vector<cv::RotatedRect> contours_rect;
        cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // cv::drawContours(img_cp, contours, -1, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("img_cp",img_cp);
        //  筛选轮廓并且画矩形把二维码框出来
        for (int i = 0; i < contours.size(); i++)
        {
            
            cv::Rect rect;
            std::vector<cv::Point> contour_a;
            double x,y;
            double height; // 长边
            double width; //短边
            double rotate_angle;
            double contour_area;
            contour_area = cv::contourArea(contours[i]);
            cv::RotatedRect rotate_rect;
            rotate_rect = cv::minAreaRect(contours[i]);
            x = rotate_rect.size.width;
            y = rotate_rect.size.height;
            if(x < y)
            {
                rotate_angle = rotate_rect.angle - 90;
            }
            else
            {
            rotate_angle = rotate_rect.angle;
                
            }
            rotate_angle *= -1; 
            height = std::max(x,y);
            width = std::min(x,y);
            if(height > 100)continue;
            if(width < 40)continue;
            // if((rotate_angle > 20 and rotate_angle <70)or(rotate_angle > 110 and rotate_angle <160))continue;
            cv::approxPolyDP(contours[i],contour_a,contours[i].size()/4,true);
            
            contours_rect.emplace_back(rotate_rect);
            contours_tag.emplace_back(contours[i]);
            contours_approx.emplace_back(contour_a);
        }
        // cv::drawContours(img_cp, contours_tag, -1, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("img_cp",img_cp);
        
        // 画出边缘矩形看看
        cv::Rect tag_rect(67,67,167,167);
        for(int i = 0; i < contours_approx.size();i++)
        {
            if(contours_approx[i].size() != 4)continue;
            cv::Point2f vertex[4];
            vertex[0] = contours_approx[i][0];
            vertex[1] = contours_approx[i][1];
            vertex[2] = contours_approx[i][2];
            vertex[3] = contours_approx[i][3];
            // 做个透视变换
            cv::Mat rotation,img_tag;
            cv::Point2f vertex_changed[4];
            vertex_changed[0] = cv::Point2f(300,0);
            vertex_changed[1] = cv::Point2f(0,0);
            vertex_changed[2] = cv::Point2f(0,300);
            vertex_changed[3] = cv::Point2f(300,300);
            rotation = cv::getPerspectiveTransform(vertex,vertex_changed);
            cv::warpPerspective(img_cp,img_tag,rotation,cv::Size(300,300));
            img_tag = img_tag(tag_rect);
            cv::imshow("img_tag",img_tag);
            cv::cvtColor(img_tag,img_tag,cv::COLOR_BGR2GRAY);
            cv::adaptiveThreshold(img_tag,img_tag,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,901,0);        // 寻找轮廓
            cv::imshow("img_tag",img_tag);
            for(int j = 0;j < 4;j++)
            {
                cv::line(img_cp,vertex[j],vertex[(j + 1) % 4],cv::Scalar(202,139,45),2);
            }
            cv::Rect rect_whole_tag;
            rect_whole_tag = cv::boundingRect(contours_approx[i]);
            Tag tag(img_tag,rect_whole_tag);
            Tags.emplace_back(tag);
        }
        

        for(int i = 0;i < Tags.size();i++)
        {
            // 第j行第k列
            for(int j = 0;j < 5;j++)
            {
                for(int k = 0; k < 5; k ++)
                {
                    cv::Rect block_rect1(0+k*33,0+j*33,33,33);
                    cv::Rect block_rect2(133-j*33,0+33*k,33,33);
                    cv::Mat rect_block1 = Tags[i].tag_region(block_rect1);
                    cv::Mat rect_block2 = Tags[i].tag_region(block_rect2);
                    // cv::cvtColor(rect_block1, rect_block1, cv::COLOR_BGR2GRAY);
                    // cv::cvtColor(rect_block2, rect_block2, cv::COLOR_BGR2GRAY);
                    Tags[i].blocks1.emplace_back(rect_block1);
                    Tags[i].blocks2.emplace_back(rect_block2);
                    int sum_pix1 = 0;
                    for(int m = 0;m < rect_block1.rows;m++)
                    {
                        uchar* data1 = rect_block1.ptr<uchar>(m);
                        for(int n = 0;n < rect_block1.cols;n++)
                        {
                            sum_pix1 += data1[n];
                        }
                    }
                    // std::cout << "sum_pix1" << ": " << sum_pix1 << std::endl;
                    if(sum_pix1 > 150000)
                    {
                        int y = Tags[i].num1.size();
                        Tags[i].num1.emplace_back(1);
                        Tags[i].Num1[y] = 1;
                    }
                    else
                    {
                        int y = Tags[i].num1.size();
                        Tags[i].num1.emplace_back(0);
                        Tags[i].Num1[y] = 0;
                    }
                    
                    ////////
                    int sum_pix2 = 0;
                    for(int m = 0;m < rect_block2.rows;m++)
                    {
                        uchar* data2 = rect_block2.ptr<uchar>(m);
                        for(int n = 0;n < rect_block2.cols;n++)
                        {
                            sum_pix2 += data2[n];
                        }
                    }
                    // std::cout << "sum_pix2" << ": " << sum_pix2 << std::endl;
                    
                    if(sum_pix2 > 150000)
                    {
                        int x = Tags[i].num2.size();
                        Tags[i].num2.emplace_back(1);
                        Tags[i].Num2[x] = 1;
                    }
                    else
                    {
                        int x = Tags[i].num2.size();
                        Tags[i].num2.emplace_back(0);
                        Tags[i].Num2[x] = 0;
                    }
                }
            }
            
            bool method_flag = false;// true 为vector false 为 数组
            // std::cout << Tags[i].num_sum << std::endl;
            if(method_flag)
            {
                Tags[i].caclnum();
                if(Tags[i].num_sum1 == 2612213 || Tags[i].num_sum2 == 2612213)
                {
                    cv::putText(img_cp,"9",cv::Point(Tags[i].rect.x -10,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 2612990 || Tags[i].num_sum2 == 2612990)
                {
                    cv::putText(img_cp,"8",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25759733 || Tags[i].num_sum2 == 25759733)
                {
                    cv::putText(img_cp,"7",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760510 || Tags[i].num_sum2 == 25760510)
                {
                    cv::putText(img_cp,"6",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760021 || Tags[i].num_sum2 == 25760021)
                {
                    cv::putText(img_cp,"5",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760240 || Tags[i].num_sum2 == 25760240)
                {
                    cv::putText(img_cp,"4",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760277 || Tags[i].num_sum2 == 25760277)
                {
                    cv::putText(img_cp,"3",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760000 || Tags[i].num_sum2 == 25760000)
                {
                    cv::putText(img_cp,"2",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].num_sum1 == 25760501 || Tags[i].num_sum2 == 25760501)
                {
                    cv::putText(img_cp,"1",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                // cv::imshow("rect_block",Tags[i].blocks[0]);
                Tags[i].blocks1.clear();
                Tags[i].num1.clear();
                Tags[i].blocks2.clear();
                Tags[i].num2.clear();
            }
            else
            {
                Tags[i].caclnum_arr();
                if(Tags[i].ID == 9)
                {
                    cv::putText(img_cp,"9",cv::Point(Tags[i].rect.x -10,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 8)
                {
                    cv::putText(img_cp,"8",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 7)
                {
                    cv::putText(img_cp,"7",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 6)
                {
                    cv::putText(img_cp,"6",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 5)
                {
                    cv::putText(img_cp,"5",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 4)
                {
                    cv::putText(img_cp,"4",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 3)
                {
                    cv::putText(img_cp,"3",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 2)
                {
                    cv::putText(img_cp,"2",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                if(Tags[i].ID == 1)
                {
                    cv::putText(img_cp,"1",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
                }
                Tags[i].blocks1.clear();
                Tags[i].num1.clear();
                Tags[i].blocks2.clear();
                Tags[i].num2.clear();
                Tags[i].similarity1.clear();
                Tags[i].similarity2.clear();
            }
            
        }
        Tags.clear();
        cv::imshow("img_cp",img_cp);
        //画出轮廓
        // cv::Point2f vertex[4];
        // for(int i = 0;i < Tags.size();i++)
        // {
        //     vertex[0] = Tags[i].rect.tl();
        //     vertex[1] = cv::Point2f(vertex[0].x+Tags[i].rect.width,vertex[0].y);
        //     vertex[2] = Tags[i].rect.br();
        //     vertex[3] = cv::Point2f(vertex[2].x-Tags[i].rect.width,vertex[2].y);
        //     Tags[i].tag_rect = cv::Rect(Tags[i].rect.x+Tags[i].rect.width/9*2,Tags[i].rect.y+Tags[i].rect.height/9*2,Tags[i].rect.width/9*5,Tags[i].rect.height/9*5);
        //     Tags[i].tag_region = img_cp(Tags[i].tag_rect);
        //     // cv::imshow("tag",Tags[i].tag_region);
        //     for(int j = 0; j < 4;j++)
        //     {
        //         cv::line(img_cp,vertex[j],vertex[(j+1)%4],cv::Scalar(0,255,0),2);
        //     }
        //     for(int k = 0; k < 5;k++)
        //     {
        //         for(int l = 0;l < 5;l++)
        //         {
        //             cv::Rect rect2(0+ Tags[i].tag_rect.width/5*l,0+ Tags[i].tag_rect.height/5*k,Tags[i].tag_rect.width/5,Tags[i].tag_rect.height/5);
        //             cv::Point2f vertex_b[4];
        //             vertex_b[0] = rect2.tl();
        //             vertex_b[1] = cv::Point2f(vertex_b[0].x+rect2.width,vertex_b[0].y);
        //             vertex_b[2] = rect2.br();
        //             vertex_b[3] = cv::Point2f(vertex_b[2].x-rect2.width,vertex_b[2].y);
        //             // for(int j = 0; j < 4;j++)
        //             // {
        //             //     cv::line(Tags[i].tag_region,vertex_b[j],vertex_b[(j+1)%4],cv::Scalar(0,0,255),2);
        //             // }
        //             // imshow("Tags[i].tag_region",Tags[i].tag_region);
        //             cv::Mat rect_block = Tags[i].tag_region(rect2);
        //             cv::cvtColor(rect_block, rect_block, cv::COLOR_BGR2GRAY);
        //             Tags[i].blocks.emplace_back(rect_block);
        //             // 判断颜色
        //             int sum_pix = 0;
        //             for(int m = 0;m < rect_block.rows;m++)
        //             {
        //                 uchar* data1 = rect_block.ptr<uchar>(m);
        //                 for(int n = 0;n < rect_block.cols;n++)
        //                 {
        //                     sum_pix += data1[n];
        //                 }
        //             }
        //             // std::cout << "NO" << k*5 + l<< ": " << sum_pix << std::endl;
        //             if(sum_pix > 35000)
        //             Tags[i].num.emplace_back(1);
        //             else
        //             Tags[i].num.emplace_back(0);
                    
        //         }
        //     }
            // Tags[i].caclnum();
            // // std::cout << Tags[i].num_sum << std::endl;
            // if(Tags[i].num_sum == 2612213)
            // {
            //     cv::putText(img_cp,"9",cv::Point(Tags[i].rect.x -10,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 2612990)
            // {
            //     cv::putText(img_cp,"8",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25759733)
            // {
            //     cv::putText(img_cp,"7",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760510)
            // {
            //     cv::putText(img_cp,"6",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760021)
            // {
            //     cv::putText(img_cp,"5",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760240)
            // {
            //     cv::putText(img_cp,"4",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760277)
            // {
            //     cv::putText(img_cp,"3",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760000)
            // {
            //     cv::putText(img_cp,"2",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // if(Tags[i].num_sum == 25760501)
            // {
            //     cv::putText(img_cp,"1",cv::Point(Tags[i].rect.x -8,Tags[i].rect.y + Tags[i].rect.height/2),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0, 0, 255), 2, 8);
            // }
            // // cv::imshow("rect_block",Tags[i].blocks[0]);
            // Tags[i].blocks.clear();
            // Tags[i].num.clear();
        // }
        
        
        // cv::imshow("img_cp", img_cp);
        // Tags.clear();
        // cv::waitKey();
    }
  
    void Tag::caclnum()
    {
        num_sum1 = 0;
        for(int m = 0;m < 5;m++)
        {
            for(int n = 0;n < 5; n++)
            {
                num_sum1 = num_sum1 +(num1[m*5+n]*std::pow(2,4-n))*std::pow(31,4-m);
            }
        }
         num_sum2 = 0;
        for(int m = 0;m < 5;m++)
        {
            for(int n = 0;n < 5; n++)
            {
                num_sum2 = num_sum2 +(num2[m*5+n]*std::pow(2,4-n))*std::pow(31,4-m);
            }
        }
    }
    void Tag::caclnum_arr()
    {
        for(int i = 0; i < 9;i++)
        {
            for(int j = 0; j < 25 ;j++)
            {
                if(NUM[i][j] == Num1[j])similarity1[i]++;
            }
        }
        for(int i = 0; i < 9;i++)
        {
            for(int j = 0; j < 25 ;j++)
            {
                if(NUM[i][j] == Num2[j])similarity2[i]++;
            }
        }
        int max_id = 0;
        int max_sim = 0;
        for(int k = 0;k < 9;k++)
        {
            if(similarity1[k] > max_sim)
            {
                max_sim = similarity1[k];
                max_id = k;
            }
            if(similarity2[k] > max_sim)
            {
                max_sim = similarity2[k];
                max_id = k;
            }
        }
        ID = max_id + 1;

    }
    TagTrack::~TagTrack() {}
}