#include <string>
#include <thread>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include <apriltag.h>
#include <tag25h9.h>

#include "move.h"

// #define RELEASE
// #define DEBUG_APRILTAG
#define DEBUG_AUTO_CRUISE
#define VISION

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480)
        : cam_id(cam_id), width(width), height(height)
    {
        std::string ip_last_segment = "15";
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
        cv::imshow("original", frame);
        if (cv::waitKey(2) == 'q')
        {
            cv::destroyAllWindows();
            exit(0);
        }
        return frame;
    }
    std::string udp_send_integrated_pipe_0;

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
};

class ControlParams
{
public:
    ControlParams(int atc, float vX, float ox,
                  float Y_Kp, float Y_Ki, float Y_Kd,
                  float R_Kp, float R_Ki, float R_Kd) : apriltagCOUNT_DOWN(atc),
                                                        vX(vX),
                                                        offsetX(ox),
                                                        Y_Kp(Y_Kp), Y_Ki(Y_Ki), Y_Kd(Y_Kd),
                                                        R_Kp(R_Kp), R_Ki(R_Ki), R_Kd(R_Kd)
    {
    }
    int apriltagCOUNT_DOWN = 10;
    float offsetX = 0.0;
    float vX = 0.0;
    float Y_Kp = 0.0, Y_Ki = 0.0, Y_Kd = 0.0;
    float R_Kp = 0.0, R_Ki = 0.0, R_Kd = 0.0;
};

struct Tasks
{
    bool task_have[5], task_done[5];
    Tasks()
    {
        memset(task_have, false, sizeof(task_have));
        memset(task_done, false, sizeof(task_done));
        task_have[0] = true;
    }
} tasks;
bool taskI_check(int i)
{
    return tasks.task_have[i] && !tasks.task_done[i];
}

class ControlRect
{
public:
    ControlRect() {}
    int width = 0, height = 0; // 图像的大小
    int left = 0, top = 0;     // 控制矩形的左上角
    int right = 0, bottom = 0; // 控制矩形的右下角
};

class CheckPoints
{
public:
    CheckPoints()
    {
        memset(time_marker, -1, sizeof(time_marker));
    }
    // time_marker[i]：标签i的时间戳：0-开始，1-结束
    // std::time_t time_marker[5][2];
    std::time_t time_marker[5];
};

class PIDController
{
    double Kp, Ki, Kd;
    double integral = 0.0;
    double previous_error = 0.0;

public:
    PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
    double control(double setpoint, double measured_value)
    {
        double error = setpoint - measured_value;

        // integral += error;
        // double derivative = error - previous_error;
        // double output = Kp*error + Ki*integral + Kd*derivative;
        // previous_error = error;

        double output = Kp * error;

        return output;
    }
};

ControlRect controlRect;
CheckPoints cp;

cv::Mat getMask(const cv::Mat &src,
                int yellowHLower = 16, int yellowHUpper = 29,
                int yellowSLower = 80, int yellowSUpper = 200,
                int yellowVLower = 110, int yellowVUpper = 220)
{
    // 转hsv，中值滤波去噪
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::medianBlur(hsv, hsv, 5);
    // 黄色阈值掩膜
    cv::Mat mask;
    cv::Scalar yellow_lower = cv::Scalar(yellowHLower, yellowSLower, yellowVLower);
    cv::Scalar yellow_upper = cv::Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    cv::inRange(hsv, yellow_lower, yellow_upper, mask);
    // 获取控制关键（腹部）
    // int width = hsv.cols / 4;
    // int height = hsv.rows / 4;
    // controlRect.left = (hsv.cols - width) / 2;
    // controlRect.top = (hsv.rows - height) / 2;
    // controlRect.right = controlRect.left + width;
    // controlRect.bottom = controlRect.top + height * 2;
    // 获取控制关键（下巴）
    float top_start = 2 / 3.0;
    float height_ratio = 1 / 20.0;
    controlRect.top = hsv.rows * top_start;
    // controlRect.top = hsv.rows * (1 - 1 / 20.0);
    controlRect.left = 0;
    controlRect.bottom = hsv.rows;
    controlRect.bottom = controlRect.top + hsv.rows * height_ratio;
    controlRect.right = hsv.cols;
    // // 显示掩膜并返回
    // cv::imshow("original_mask", mask);
    return mask;
}
void showHighlightMask(const cv::Mat &img, const cv::Mat &mask,
                       int B = 255, int G = 0, int R = 0,
                       double alpha = 0.7, double beta = 1.0)
{
    // 创建一个与原图像大小和类型相同的纯色图像
    cv::Mat color(img.size(), img.type(), cv::Scalar(B, G, R));
    // 使用掩膜将纯色图像应用到原图像上
    cv::Mat masked_color;
    cv::bitwise_and(color, color, masked_color, mask);
    // 将纯色的部分和原图像的其余部分组合在一起
    cv::Mat result;
    cv::addWeighted(img, beta, masked_color, alpha, 0.0, result);

    // 在图像中央画一个矩形
    // 绘制控制关键
    cv::Point topLeft(controlRect.left, controlRect.top);
    cv::Point bottomRight(controlRect.right, controlRect.bottom);
    cv::rectangle(result, topLeft, bottomRight, cv::Scalar(255 - B, 255 - G, 255 - R), 4);

    // 显示结果
    cv::imshow("Result", result);
    cv::moveWindow("Result", 900, 500);
}

std::pair<int, int> countWhitePixelsOnSides(const cv::Mat &mask, const cv::Rect &rect)
{
    cv::Mat roi = mask(rect); // Extract the region of interest
    std::cout << "ROI size: " << roi.size() << std::endl;
    std::cout << roi.rows << " " << roi.cols << std::endl;
    // Calculate the midpoint of the width
    int midPoint = roi.cols / 2;
    // Split the ROI into left and right halves
    cv::Mat leftHalf = roi(cv::Range(0, roi.rows), cv::Range(0, midPoint));
    cv::Mat rightHalf = roi(cv::Range(0, roi.rows), cv::Range(midPoint, roi.cols));
    // Count the white pixels in each half
    int leftCount = cv::countNonZero(leftHalf);
    int rightCount = cv::countNonZero(rightHalf);
    return {leftCount, rightCount};
}

#ifdef DEBUG_AUTO_CRUISE

bool cp_markerI_first(int i)
{
    return cp.time_marker[i] > -1;
}
bool cp_markerI_out_s(int i, int s)
{
    return std::difftime(std::time(nullptr), cp.time_marker[i]) >= s;
}

float calErr(const cv::Mat &img)
{
    cv::Rect rect(controlRect.left, controlRect.top, controlRect.right - controlRect.left, controlRect.bottom - controlRect.top);
    cv::Mat subImg = img(rect);
    std::vector<cv::Point2f> points;
    // 遍历每一行
    for (int y = 0; y < subImg.rows; y++)
    {
        int count = 0;
        int sum = 0;
        // 遍历每一列
        for (int x = 0; x < subImg.cols; x++)
        {
            // 如果这个像素是白色的
            if (subImg.at<uchar>(y, x) > 0)
            {
                sum += x;
                count++;
            }
        }
        // 计算这一行的像素中点
        if (count > 0)
        {
            points.push_back(cv::Point2f(sum / count, y));
        }
    }
    // 计算x坐标的中心
    float sumX = 0;
    for (const cv::Point2f &point : points)
    {
        sumX += point.x;
    }
    float centerX = sumX / points.size();

    // cv::Mat subImg_show = subImg.clone();
    // 创建一个与二值化图像同样大小的三通道图像
    cv::Mat color_img = cv::Mat::zeros(subImg.size(), CV_8UC3);
    // 遍历二值化图像的每一个像素
    for (int i = 0; i < subImg.rows; i++)
    {
        for (int j = 0; j < subImg.cols; j++)
        {
            // 如果二值化图像的像素值为255（白色），则设置彩色图像的颜色为白色
            if (subImg.at<uchar>(i, j) == 255)
            {
                color_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            // 如果二值化图像的像素值为0（黑色），则设置彩色图像的颜色为黑色
            else
            {
                color_img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    // 计算图像的中心
    int imgCenterX = color_img.cols / 2;
    int imgCenterY = color_img.rows / 2;
    // 在图像上绘制x的中心
    cv::circle(color_img, cv::Point(centerX, imgCenterY), 5, cv::Scalar(0, 0, 255), -1);
    // 在图像上绘制图像的中心
    cv::circle(color_img, cv::Point(imgCenterX, imgCenterY), 5, cv::Scalar(0, 255, 0), -1);
    std::cout << "deltaX: " << centerX - imgCenterX << std::endl;

    // 显示图像
    cv::imshow("Image with Line", color_img);
    cv::waitKey(2);
    return centerX - imgCenterX;
}

int apriltagDetector(const cv::Mat &img)
{
    apriltag_family_t *tf = tag25h9_create();             // 创建一个tag16h5家族的实例
    apriltag_detector_t *td = apriltag_detector_create(); // 创建一个AprilTag检测器的实例
    apriltag_detector_add_family(td, tf);                 // 标签族添加到检测器中
    cv::Mat gray;                                         // apriltag只能处理灰度图像
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // Mat图像转换为AprilTag库可以处理的image_u8_t格式
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};
    // Detect AprilTags in the image
    zarray_t *detections = apriltag_detector_detect(td, &im);
    // 只获取第一个检测到的标签
    int res = -1;
    if (zarray_size(detections) > 0)
    {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det); // 只获取第一个检测到的标签
        res = det->id;
    }
    // Clean up
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag25h9_destroy(tf);
    return res;
}

// 不错的参数
ControlParams params(
    10,            // apriltagCOUNT_DOWN，计数器，每x帧检测一次
    0.15,          // vX，前进速度
    30,            // offsetX，偏移量
    0.00002, 0, 0, // Y_Kp, Y_Ki, Y_Kd：Y轴PID参数
    -0.015, 0, 0); // R_Kp, R_Ki, R_Kd：R轴PID参数

PIDController pidY(params.Y_Kp, params.Y_Ki, params.Y_Kd);
PIDController pidR(params.R_Kp, params.R_Ki, params.R_Kd);

void point_walking(Custom &custom, const cv::Mat &mask, float y_offset)
{
    auto err = calErr(mask); // 计算误差
    double offsetX = params.offsetX;
    if (err < 0)
        err += offsetX;
    else
        err -= offsetX;
    float Youtput = y_offset;
    float Routput = pidR.control(0, err);
    std::cout << "Error: " << err << std::endl;
    std::cout << "Youtput: " << Youtput << std::endl;
    std::cout << "Routput: " << Routput << std::endl;
    custom.setVelocity(params.vX, Youtput, Routput);
}
void out_area(Camera &cam, Custom &custom, int taskID)
{ // 离开任务区
    std::cout << "\033[1;31mOut of the area" << "\033[0m" << std::endl;
    // 先停下来稳定
    custom.setVelocity(0, 0, 0);
    sleep(1);
    // 前进
    if (taskID == 1)
    {
        custom.setVelocity(0.2, 0, 0);
        sleep(2);
    }
    else
    {
        custom.setVelocity(0, 0, 0.25);
        sleep(2);
        custom.setVelocity(0.2, 0, 0);
        sleep(3);
    }

    custom.setVelocity(0, 0, 0);
    sleep(1);
    int marker_id = -1;
    cv::Mat mask;
    cam.reopen();
    while (1)
    {
        cv::Mat img = cam.getFrame();
        marker_id = apriltagDetector(img);
        if (marker_id == -1)
        {
            break;
        }
        mask = getMask(img, 10, 40, 50, 255, 0, 255);
        if (taskID == 2)
        {
            point_walking(custom, mask, -0.2);
        }
        else
        {
            point_walking(custom, mask, -0.35);
        }
    }
}
bool now_time_out(std::time_t t, int s)
{
    return std::difftime(std::time(nullptr), t) >= s;
}

int main()
{
    tasks.task_have[1] = true;
    tasks.task_have[2] = true;
    tasks.task_have[3] = true;
    tasks.task_have[4] = true;

    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);

    // 先出启动区
    custom.setVelocity(0.0, 0.45, 0.0);
    sleep(2);
    custom.setVelocity(0.0, 0.0, 0.0);
    sleep(1);
    // 视觉处理控制运动
    cv::Mat img, mask;
    Camera cam(2);
    std::time_t time_start = std::time(nullptr);
    while (1)
    {
        img = cam.getFrame();
        int apriltagDetectedID = apriltagDetector(img);
        if (apriltagDetectedID != -1)
        {
            std::cout << "\033[1;31mAprilTag detected: " << apriltagDetectedID << "\033[0m" << std::endl;
            if (apriltagDetectedID == 0)
            {
                if (now_time_out(time_start, 60))
                {
                    tasks.task_done[0] = true;
                    custom.setVelocity(0, 0, 0);
                    sleep(1);
                    custom.setVelocity(0, -0.3, 0);
                    sleep(3);
                    custom.setVelocity(0, 0, 0);
                    sleep(1);
                    std::cout << "\033[1;32mCongratulations, missions completed!\033[0m" << std::endl;
                    break;
                }
            }
            else
            {
                // if (apriltagDetectedID == 2) {
                //     out_area(cam, custom, apriltagDetectedID);
                // }
                if (taskI_check(apriltagDetectedID))
                {
                    if (apriltagDetectedID)
                        if (cp.time_marker[apriltagDetectedID] == -1)
                        {
                            custom.setVelocity(0, 0, 0);
                            sleep(1);
                            cp.time_marker[apriltagDetectedID] = std::time(nullptr);
                            if (apriltagDetectedID == 1 || apriltagDetectedID == 2)
                            {
                                // 进环
                                std::cout << "\033[1;31mReady for going in 1/2" << "\033[0m" << std::endl;
                                // 直走
                                std::cout << "\033[1;31mGo straight" << "\033[0m" << std::endl;
                                if (apriltagDetectedID == 1)
                                {
                                    custom.setVelocity(0.25, 0, 0);
                                    sleep(2);
                                    // 左平移
                                    std::cout << "\033[1;31mGo left" << "\033[0m" << std::endl;
                                    custom.setVelocity(0, 0.25, 0);
                                    sleep(1);
                                }
                                else
                                {
                                    custom.setVelocity(0.25, 0, 0);
                                    sleep(3);
                                }
                                // 停一会
                                custom.setVelocity(0, 0, 0);
                                sleep(1);
                                // 左转进圈
                                std::cout << "\033[1;31mGo left in circle" << "\033[0m" << std::endl;
                                // 左转
                                custom.setVelocity(0, 0, 0.4);
                                sleep(1);
                                // 直走
                                custom.setVelocity(0.4, 0, 0);
                                sleep(1);
                            }
                            if (apriltagDetectedID == 3 || apriltagDetectedID == 4)
                            {
                                // 直走进环
                                std::cout << "\033[1;31m3/4: Go in" << "\033[0m" << std::endl;
                                if (apriltagDetectedID == 3)
                                {
                                    // 左转修正角度
                                    custom.setVelocity(0, 0, 0.1);
                                    sleep(1);
                                } else {
                                    // 右转修正角度
                                    custom.setVelocity(0, 0, -0.1);
                                    sleep(1);
                                }
                                custom.setVelocity(0.2, 0, 0);
                                sleep(2);
                            }
                            custom.setVelocity(0, 0, 0);
                            sleep(1);
                            cam.reopen();
                        }
                        else
                        {
                            if (cp_markerI_out_s(apriltagDetectedID, 10))
                            {
                                tasks.task_done[apriltagDetectedID] = true;
                                if (apriltagDetectedID == 1 || apriltagDetectedID == 2)
                                    out_area(cam, custom, apriltagDetectedID);
                                else
                                {
                                    custom.setVelocity(0, 0, 0);
                                    sleep(1);
                                    // 左转修正角度
                                    custom.setVelocity(0, 0, -0.1);
                                    sleep(1);
                                    custom.setVelocity(0.2, 0, 0);
                                    sleep(2);
                                    custom.setVelocity(0, 0, 0);
                                    sleep(1);
                                    cam.reopen();
                                    img = cam.getFrame();
                                }
                            }
                        }
                }
            }
        }
        mask = getMask(img, 10, 40, 50, 255, 0, 255);
        showHighlightMask(img, mask, 0, 255, 0);
        point_walking(custom, mask, 0);
    }

    // mainControl.join();
    return 0;
}
#endif
