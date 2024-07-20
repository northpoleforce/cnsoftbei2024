#include <string>
#include <thread>
#include <numeric>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "move.h"

using namespace cv;
using namespace std;

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480)
        : cam_id(cam_id), width(width), height(height)
    {
        std::string ip_last_segment = "123";
        std::string udpstr_prev_data = "udpsrc address=192.168.123." + ip_last_segment + " port=";
        std::vector<int> udp_port = {9201, 9202, 9203, 9204, 9205};
        std::string udpstr_behind_data = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
        udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
        std::cout << udp_send_integrated_pipe_0 << std::endl;
        cap.open(udp_send_integrated_pipe_0);
    }
    ~Camera()
    {
        cv::destroyAllWindows();
        cap.release();
    }
    void reopen()
    {
        cap.release();
        cap.open(udp_send_integrated_pipe_0);
    }
    cv::Mat getFrame()
    {
        cv::Mat frame;
        while (!cap.read(frame))
            ;
        cv::resize(frame, frame, cv::Size(width, height));
        if (cam_id == 1)
        {
            cv::flip(frame, frame, -1);
        }
        // cv::imshow("frame", frame);
        cv::waitKey(1);
        return frame;
    }
    std::string udp_send_integrated_pipe_0;

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
};

class HSVMaskGenerator
{
public:
    HSVMaskGenerator(int colorHLower, int colorHUpper,
                     int colorSLower, int colorSUpper,
                     int colorVLower, int colorVUpper)
        : colorHLower(colorHLower), colorHUpper(colorHUpper),
          colorSLower(colorSLower), colorSUpper(colorSUpper),
          colorVLower(colorVLower), colorVUpper(colorVUpper) {}

    cv::Mat getMask(cv::Mat src)
    {
        // 转hsv，中值滤波去噪
        cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
        cv::medianBlur(src, src, 5);
        // 阈值掩膜
        cv::Mat mask;
        const cv::Scalar color_lower = cv::Scalar(colorHLower, colorSLower, colorVLower);
        const cv::Scalar color_upper = cv::Scalar(colorHUpper, colorSUpper, colorVUpper);
        cv::inRange(src, color_lower, color_upper, mask);
        // 单通道图像的二值图像
        return mask;
    }

private:
    int colorHLower, colorHUpper;
    int colorSLower, colorSUpper;
    int colorVLower, colorVUpper;
};
class BGRMaskGenerator
{
public:
    BGRMaskGenerator(int colorBLower, int colorBUpper,
                     int colorGLower, int colorGUpper,
                     int colorRLower, int colorRUpper)
        : colorBLower(colorBLower), colorBUpper(colorBUpper),
          colorGLower(colorGLower), colorGUpper(colorGUpper),
          colorRLower(colorRLower), colorRUpper(colorRUpper) {}

    cv::Mat getMask(cv::Mat src)
    {
        // 中值滤波去噪
        cv::medianBlur(src, src, 5);
        // 阈值掩膜
        cv::Mat mask;
        const cv::Scalar color_lower = cv::Scalar(colorBLower, colorGLower, colorRLower);
        const cv::Scalar color_upper = cv::Scalar(colorBUpper, colorGUpper, colorRUpper);
        cv::inRange(src, color_lower, color_upper, mask);
        return mask;
    }

private:
    int colorBLower, colorBUpper;
    int colorGLower, colorGUpper;
    int colorRLower, colorRUpper;
};
class GRAYMaskGenerator
{
public:
    GRAYMaskGenerator(int grayLower, int grayUpper)
        : grayLower(grayLower), grayUpper(grayUpper) {}

    cv::Mat getMask(cv::Mat src)
    {
        // 中值滤波去噪
        cv::medianBlur(src, src, 5);
        // 阈值掩膜
        cv::Mat mask;
        cv::inRange(src, grayLower, grayUpper, mask);
        return mask;
    }

private:
    int grayLower, grayUpper;
};

// 找到最大的连通区域
int dfs(short x, short y, cv::Mat &img, std::vector<std::vector<int>> &amount, std::vector<std::vector<bool>> &visited)
{
    if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
        return 0;
    visited[x][y] = true;
    amount[x][y] = 1;
    amount[x][y] += dfs(x + 1, y, img, amount, visited);
    amount[x][y] += dfs(x - 1, y, img, amount, visited);
    amount[x][y] += dfs(x, y + 1, img, amount, visited);
    amount[x][y] += dfs(x, y - 1, img, amount, visited);
    return amount[x][y];
}
// 染色
void dye(short x, short y, cv::Mat &img, std::vector<std::vector<bool>> &visited, cv::Mat &mask)
{
    // 递归终止条件
    if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
        return;
    // 染色
    visited[x][y] = true;
    mask.at<uchar>(x, y) = 255;
    // 递归
    dye(x + 1, y, img, visited, mask);
    dye(x - 1, y, img, visited, mask);
    dye(x, y + 1, img, visited, mask);
    dye(x, y - 1, img, visited, mask);
}
cv::Mat createCustomMask(const cv::Mat &inputImage)
{
    cv::Mat cloneImage = inputImage.clone();
    // 复制cloneImage等大小的二维数组
    std::vector<std::vector<int>> amount(cloneImage.rows, std::vector<int>(cloneImage.cols));
    std::vector<std::vector<bool>> visited(cloneImage.rows, std::vector<bool>(cloneImage.cols));
    // 从左上角开始遍历
    int maxAmount = 0;
    int maxAmountX = 0, maxAmountY = 0;
    for (int i = 0; i < cloneImage.rows; i++)
    {
        for (int j = 0; j < cloneImage.cols; j++)
        {
            if (cloneImage.at<uchar>(i, j) == 255 && !visited[i][j])
            {
                dfs(i, j, cloneImage, amount, visited);
                if (amount[i][j] > maxAmount)
                {
                    maxAmount = amount[i][j];
                    maxAmountX = i;
                    maxAmountY = j;
                }
            }
        }
    }
    cv::Mat outputImage = cv::Mat::zeros(cloneImage.rows, cloneImage.cols, CV_8UC1);
    // 清空visited数组
    for (int i = 0; i < cloneImage.rows; i++)
    {
        for (int j = 0; j < cloneImage.cols; j++)
        {
            visited[i][j] = false;
        }
    }
    dye(maxAmountX, maxAmountY, cloneImage, visited, outputImage);
    return outputImage;
}

// 线性拟合函数
pair<double, double> liner(const Mat& img, Mat& result) {
    // 获取图像尺寸
    int rows = img.rows;
    int cols = img.cols;
    // 获取非零像素的位置
    vector<Point> points;
    findNonZero(img, points);
    if (points.size() < 2) {
        return make_pair(0, 0);
    }
    // 将非零像素的坐标分为 x 和 y 坐标
    vector<double> x_points, y_points;
    for (const Point& p : points) {
        x_points.push_back(p.y); // 注意：在OpenCV中，Point的x是列，y是行
        y_points.push_back(p.x);
    }
    // 线性拟合
    Vec4f fitted_line;
    fitLine(points, fitted_line, DIST_L2, 0, 0.01, 0.01);
    double vx = fitted_line[0], vy = fitted_line[1], x0 = fitted_line[2], y0 = fitted_line[3];
    // 计算拟合直线的端点
    Point point1, point2;
    point1.x = int((0 - y0) * vx / vy + x0);
    point1.y = 0;
    point2.x = int((rows - y0) * vx / vy + x0);
    point2.y = rows;
    // 绘制中线和拟合直线
    line(result, Point(cols / 2, 0), Point(cols / 2, rows), Scalar(0, 0, 255), 2);
    line(result, point1, point2, Scalar(0, 255, 0), 2);
    // 计算和返回结果
    double distance = cols / 2 - (rows * vx / vy + x0);
    double intercept = y0 - vx * x0 / vy;
    return make_pair(distance, intercept);
}

int main()
{
    Camera cam(2);
    HSVMaskGenerator HSV_mask_generator(0, 180, 0, 255, 0, 255);
    BGRMaskGenerator BGR_mask_generator(0, 100, 0, 100, 0, 100);
    GRAYMaskGenerator gray_mask_generator(0, 80);
    while (1)
    {
        cv::Mat frame = cam.getFrame();

        // 掩膜生成
        cv::Mat maskHSV = HSV_mask_generator.getMask(frame.clone());
        cv::Mat maskBGR = BGR_mask_generator.getMask(frame.clone());
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::Mat maskGRAY = gray_mask_generator.getMask(gray_frame);
        cv::Mat intersection;
        cv::bitwise_and(maskHSV, maskBGR, intersection);
        cv::bitwise_and(intersection, maskGRAY, intersection);

        cv::Mat customMask = createCustomMask(intersection);
        cv::imshow("customMask", customMask);
        cv::Mat result_color = frame.clone();
        liner(customMask, result_color);
        cv::imshow("result_color", result_color);

        // {
        //     // 格式转换
        //     cv::Mat maskHSV_color, maskBGR_color,
        //         maskGRAY_color, gray_frame_color,
        //         intersection_color;
        //     cv::cvtColor(maskHSV, maskHSV_color, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(maskBGR, maskBGR_color, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(gray_frame, gray_frame_color, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(maskGRAY, maskGRAY_color, cv::COLOR_GRAY2BGR);
        //     cv::cvtColor(intersection, intersection_color, cv::COLOR_GRAY2BGR);
        //     {
        //         // 水平拼接frame和intersection_color
        //         cv::Mat horizontalImg0;
        //         cv::hconcat(frame, intersection_color, horizontalImg0);
        //         // 画线分隔每个画面
        //         int lineThickness = 2;
        //         cv::Scalar lineColor(0, 0, 255); // 红色
        //         cv::line(horizontalImg0, cv::Point(horizontalImg0.cols / 2, 0), cv::Point(horizontalImg0.cols / 2, horizontalImg0.rows), lineColor, lineThickness);
        //         // 在每个画面上显示文字备注
        //         cv::Scalar textColor(0, 0, 255); // 红色
        //         cv::putText(horizontalImg0, "Before", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         cv::putText(horizontalImg0, "After", cv::Point(horizontalImg0.cols / 2 + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         cv::imshow("before & after", horizontalImg0);
        //     }
        //     {
        //         // 水平拼接maskHSV_color和maskBGR_color
        //         cv::Mat horizontalImg1;
        //         cv::hconcat(maskHSV_color, maskBGR_color, horizontalImg1);
        //         // 水平拼接gray_frame_color和maskGRAY_color
        //         cv::Mat horizontalImg2;
        //         cv::hconcat(gray_frame_color, maskGRAY_color, horizontalImg2);
        //         // 垂直拼接verticalImg1和horizontalImg2
        //         cv::Mat verticalImg1;
        //         cv::vconcat(horizontalImg1, horizontalImg2, verticalImg1);
        //         // 画线分隔每个画面
        //         int lineThickness = 2;
        //         cv::Scalar lineColor(0, 0, 255); // 红色
        //         cv::line(verticalImg1, cv::Point(verticalImg1.cols / 2, 0), cv::Point(verticalImg1.cols / 2, verticalImg1.rows), lineColor, lineThickness);
        //         cv::line(verticalImg1, cv::Point(0, verticalImg1.rows / 2), cv::Point(verticalImg1.cols, verticalImg1.rows / 2), lineColor, lineThickness);
        //         // 在每个画面上显示红色的文字备注
        //         cv::Scalar textColor(0, 0, 255); // 红色
        //         cv::putText(verticalImg1, "Mask HSV", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         cv::putText(verticalImg1, "Mask BGR", cv::Point(verticalImg1.cols / 2 + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         cv::putText(verticalImg1, "Gray Frame", cv::Point(10, verticalImg1.rows / 2 + 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         cv::putText(verticalImg1, "Mask GRAY", cv::Point(verticalImg1.cols / 2 + 10, verticalImg1.rows / 2 + 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //         // 显示拼接后的图像
        //         cv::imshow("all masks", verticalImg1);
        //     }
        // }

        // // 使用Canny算法进行边缘检测
        // cv::Mat edges;
        // cv::Canny(customMask, edges, 50, 200, 5);
        // // cv::rectangle(edges, cv::Point(0, 0), cv::Point(edges.cols - 1, edges.rows - 1), cv::Scalar(255), 1);
        // // 绘制上边框（不包括端点）
        // // cv::line(edges, cv::Point(1, 0), cv::Point(edges.cols - 2, 0), cv::Scalar(255), 1);
        // // 绘制下边框（不包括端点）
        // cv::line(edges, cv::Point(1, edges.rows - 1), cv::Point(edges.cols - 2, edges.rows - 1), cv::Scalar(255), 1);
        // // 绘制左边框（不包括端点）
        // // cv::line(edges, cv::Point(0, 1), cv::Point(0, edges.rows - 2), cv::Scalar(255), 1);
        // // 绘制右边框（不包括端点）
        // // cv::line(edges, cv::Point(edges.cols - 1, 1), cv::Point(edges.cols - 1, edges.rows - 2), cv::Scalar(255), 1);
        // cv::imshow("Canny Edge Detection", edges);

        // // 过滤轮廓
        // std::vector<std::vector<cv::Point>> contours;
        // std::vector<cv::Vec4i> hierarchy;
        // // 创建掩膜
        // cv::Mat maskCounters = cv::Mat::zeros(edges.size(), CV_8UC1);
        // {
        //     // 查找轮廓
        //     cv::findContours(edges, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        //     // 设置最小面积阈值
        //     double min_area = 10000;
        //     // 设置最小周长阈值
        //     double min_perimeter = 1000;
        //     // 过滤轮廓
        //     std::vector<std::vector<cv::Point>> filtered_contours;
        //     for (auto &contour : contours)
        //     {
        //         double area = cv::contourArea(contour);
        //         if (area < min_area)
        //             continue;
        //         double perimeter = cv::arcLength(contour, true);
        //         if (perimeter < min_perimeter)
        //             continue;

        //         vector<cv::Point> approx;
        //         approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);
        //         std::cout << "approx.size: " << approx.size() << std::endl;
        //         if (approx.size() == 4)
        //         {
        //             filtered_contours.push_back(contour);
        //             // 绘制approx
        //             cv::drawContours(maskCounters, filtered_contours, -1, cv::Scalar(255), 2);
        //         }
        //         // filtered_contours.push_back(contour);
        //     }
        //     cv::drawContours(maskCounters, filtered_contours, -1, cv::Scalar(255), 1);
        //     cv::rectangle(maskCounters, cv::Point(0, 0), cv::Point(maskCounters.cols - 1, maskCounters.rows - 1), cv::Scalar(0), 1);
        //     // 显示掩膜
        //     cv::imshow("maskCounters", maskCounters);
        // }

        // // sobel算子
        // {
        //     // 初始化变量
        //     cv::Mat grad;
        //     int scale = 1;
        //     int delta = 0;
        //     int ddepth = CV_16S;
        //     // Gradient X
        //     cv::Mat grad_x;
        //     cv::Sobel(intersection, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
        //     // Gradient Y
        //     cv::Mat grad_y;
        //     cv::Sobel(intersection, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
        //     // Converting back to CV_8U depth
        //     cv::Mat abs_grad_x, abs_grad_y;
        //     cv::convertScaleAbs(grad_x, abs_grad_x);
        //     cv::convertScaleAbs(grad_y, abs_grad_y);
        //     // Total Gradient (approximate)
        //     cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
        //     // 显示结果
        //     cv::imshow("Sobel Edge Detection", grad);
        // }

        // // 使用HoughLinesP检测直线
        // std::vector<cv::Vec4i> lines;
        // cv::HoughLinesP(maskCounters, lines, 1, CV_PI / 180, 50, 20, 20);
        // cv::Mat result = frame.clone();
        // // 输出直线的数量
        // std::cout << "lines.size(): " << lines.size() << std::endl;
        // for (size_t i = 0; i < lines.size(); i++)
        // {
        //     // 输出直线的端点坐标
        //     std::cout << "lines[" << i << "]: " << lines[i] << std::endl;
        //     cv::Vec4i l = lines[i];
        //     cv::line(result, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        // }
        // {
        //     // 把maskCounters和result水平拼接
        //     cv::Mat maskCounters_color;
        //     cv::cvtColor(maskCounters, maskCounters_color, cv::COLOR_GRAY2BGR);
        //     cv::Mat horizontalImg3;
        //     cv::hconcat(maskCounters_color, result, horizontalImg3);
        //     // 画线分隔每个画面
        //     int lineThickness = 2;
        //     cv::Scalar lineColor(0, 0, 255); // 红色
        //     cv::line(horizontalImg3, cv::Point(horizontalImg3.cols / 2, 0), cv::Point(horizontalImg3.cols / 2, horizontalImg3.rows), lineColor, lineThickness);
        //     // 在每个画面上显示文字备注
        //     cv::Scalar textColor(0, 0, 255); // 红色
        //     cv::putText(horizontalImg3, "maskCounters", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //     cv::putText(horizontalImg3, "Result", cv::Point(horizontalImg3.cols / 2 + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, textColor);
        //     cv::imshow("maskCounters & result", horizontalImg3);
        // }

        // // 检测角点
        // std::vector<cv::Point2f> corners;
        // cv::goodFeaturesToTrack(intersection, corners, 100, 0.1, 50);
        // // 转换角点到关键点
        // std::vector<cv::KeyPoint> keypoints;
        // for (auto &point : corners)
        // {
        //     keypoints.push_back(cv::KeyPoint(point, 1));
        // }
        // // 调整关键点的大小并绘制实心的红色关键点
        // cv::Mat img_keypoints = frame.clone();
        // for (auto &keypoint : keypoints)
        // {
        //     int radius = 5; // 你可以根据需要调整这个值
        //     cv::circle(img_keypoints, keypoint.pt, radius, cv::Scalar(0, 255, 0), -1);
        // }

        // // 绘制关键点
        // cv::Mat img_keypoints;
        // cv::drawKeypoints(intersection, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // // cv::drawKeypoints(intersection, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // 显示图像
        // cv::imshow("Keypoints", img_keypoints);

        cv::waitKey(1);
    }

    return 0;
}