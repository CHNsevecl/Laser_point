#include "rect.hpp"

cv::RotatedRect Find_Rect(cv::Mat& BGR_frame){

    cv::Mat Gray_frame,Binary_frame;

    cv::cvtColor(BGR_frame, Gray_frame, cv::COLOR_BGR2GRAY);

    // cv::threshold(Gray_frame, Binary_frame, 75, 255, cv::THRESH_BINARY_INV);
    cv::adaptiveThreshold(Gray_frame, Binary_frame, 255, 
                cv::ADAPTIVE_THRESH_GAUSSIAN_C,  // 高斯加权平均
                cv::THRESH_BINARY_INV, 
                25,  // 邻域大小（奇数，越大越不敏感）
                10);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));

    // 1. 开运算：先腐蚀后膨胀（去除白色噪点）
    cv::morphologyEx(Binary_frame, Binary_frame, cv::MORPH_OPEN, kernel);

    // 2. 闭运算：先膨胀后腐蚀（填充黑色空洞）
    cv::morphologyEx(Binary_frame, Binary_frame, cv::MORPH_CLOSE, kernel);
    //=============================================

    //=====================找轮廓==================
    std::vector<std::vector<cv::Point>> contours;
    findContours(Binary_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 存储检测到的方框
    cv::RotatedRect final_rect;

    // 遍历每个轮廓
    int maxArea = 0;
    for (const auto& contour : contours) {
        // 1. 面积过滤
        double area = contourArea(contour);
        
        if (!(area > maxArea)){
            continue;
        }

        maxArea = area;
        
        // 2. 多边形逼近
        std::vector<cv::Point> approx;
        double peri = arcLength(contour, true);
        approxPolyDP(contour, approx, 0.02 * peri, true);
        
        // 3. 必须是四边形
        if (approx.size() != 4) continue;
        
        // 4. 检查是否为矩形（可选）
        cv::Rect rect = boundingRect(approx);//bounding 表示外围，可能（当矩形旋转时）存在空白
        double aspectRatio = (double)rect.width / rect.height;
        if (aspectRatio < 0.7 || aspectRatio > 1.3) continue;
        
        // 通过筛选，保存结果
        final_rect = minAreaRect(approx);//minArea 最小面积，去除空白，有角度
    }
    //============================================

    return final_rect;
}

cv::Mat createRotatedRectMask(int img_width, int img_height, cv::RotatedRect rotated_rect) {
    // 创建全黑掩码
    cv::Mat mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
    
    // 获取旋转矩形的4个顶点
    cv::Point2f vertices[4];
    rotated_rect.points(vertices);
    
    // 将顶点转换为Point类型
    std::vector<cv::Point> pts;
    for (int i = 0; i < 4; i++) {
        pts.push_back(cv::Point(static_cast<int>(vertices[i].x), 
                                static_cast<int>(vertices[i].y)));
    }
    
    // 填充多边形
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{pts}, cv::Scalar(255));
    
    return mask;
}