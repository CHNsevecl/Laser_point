#ifndef RECT_HPP
#define RECT_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

cv::RotatedRect Find_Rect(cv::Mat& BGR_frame);
cv::Mat createRotatedRectMask(int img_width, int img_height, cv::RotatedRect rotated_rect);

#endif