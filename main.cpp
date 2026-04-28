#include <opencv2/opencv.hpp>
#include <iostream>
#include "uart.hpp"
#include <cstdlib>

int main(){
    setenv("DISPLAY",":0",1);

    //================初始化摄像头================
    std::string pipeline = 
    {
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw,format=NV12,width=640,height=480,framerate=120/1 ! "
        "appsink drop=true max-buffers=1 sync=false"
    };

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()){
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }
    //================================

    //============初始化UART============
    UART uart;
    if (!uart.init("/dev/serial0", 115200)) {
        return -1;
    }

    //================================

    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int count = 0;
    std::vector<cv::Point> blue_per_round_points(5);
    std::vector<cv::Point> red_per_round_points(5);

    //============处理视频流============
    while (true){
        cv::Mat frame_BGR;
        cv::Mat frame_HSV;
        cap >> frame_BGR;

        if (frame_BGR.empty()){
            std::cerr << "Error reading frame" << std::endl;
            break;
        }
        
        //===============二值化================
        cv::cvtColor(frame_BGR, frame_BGR, cv::COLOR_YUV2BGR_NV12);
        cv::cvtColor(frame_BGR, frame_HSV, cv::COLOR_BGR2HSV);



        cv::Mat mask1, mask2, mask_red;
        cv::inRange(frame_HSV, cv::Scalar(0, 20, 200), cv::Scalar(20, 255, 255), mask1);
        cv::inRange(frame_HSV, cv::Scalar(160, 20, 200), cv::Scalar(179, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask_red);

        cv::Mat mask_blue;
        cv::inRange(frame_HSV, cv::Scalar(100, 30, 230), cv::Scalar(130, 255, 255), mask_blue);
    
        // 只填充微小空洞，不去主动腐蚀
        cv::Mat kernel_red = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::Mat kernel_blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel_red);
        cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel_blue);
        //====================================


        std::vector<cv::Point> all_point;
        all_point.reserve(2);
        //==============红点==================
        std::vector<std::vector<cv::Point>> contours_red;
        cv::findContours(mask_red, contours_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto& contour : contours_red) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                // center 就是激光中心
                cv::circle(frame_BGR, center,2,cv::Scalar(0,0,255),-1);
                all_point.push_back(center);
                break;
            }
        }
        //===============蓝点===================
        std::vector<std::vector<cv::Point>> contours_blue;
        cv::findContours(mask_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours_blue) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                // center 就是激光中心
                cv::circle(frame_BGR, center,2,cv::Scalar(255,0,0),-1);
                all_point.push_back(center);
                break;
            }
        }
        //======================================
        if ((all_point[0].x == 0 && all_point[0].y ==0) || (all_point[1].x == 0 && all_point[1].y ==0)){
            send_direction_to_MCU(uart,cv::Point(0, 0));
        }
        else{
            int dx = all_point[1].x - all_point[0].x;
            int dy = all_point[1].y - all_point[0].y;

            if(abs(dx) <= 5){
                dx = 0;
            }
            if(abs(dy) <= 5){
                dy = 0;
            }
            send_direction_to_MCU(uart,cv::Point(dx,dy));
        }
     
        
        cv::imshow("bgr",frame_BGR);
        cv::imshow("blue",mask_blue);
        cv::imshow("red",mask_red);
        
        if (cv::waitKey(1) == 27){
            break;
        }
    }
    uart.close_port();
    return 0;
}