#include <string>
#include <thread>
#include <numeric>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <tag25h9.h>
#include <apriltag.h>
#include <opencv2/opencv.hpp>

#include "move.h"

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
class ImageProcessor
{
public:
    ImageProcessor() {}
    cv::Mat createCustomMask(const cv::Mat &inputImage)
    {
        cv::Mat cloneImage = inputImage.clone();
        amount = std::vector<std::vector<int>>(cloneImage.rows, std::vector<int>(cloneImage.cols));
        visited = std::vector<std::vector<bool>>(cloneImage.rows, std::vector<bool>(cloneImage.cols));
        int maxAmount = 0;
        int maxAmountX = 0, maxAmountY = 0;
        for (int i = 0; i < cloneImage.rows; i++)
        {
            for (int j = 0; j < cloneImage.cols; j++)
            {
                if (cloneImage.at<uchar>(i, j) == 255 && !visited[i][j])
                {
                    dfs(i, j, cloneImage);
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
        resetVisited(cloneImage.rows, cloneImage.cols);
        dye(maxAmountX, maxAmountY, cloneImage, outputImage);
        return outputImage;
    }
    std::pair<double, std::pair<bool, double>> liner(const cv::Mat &img, cv::Mat &result)
    {
        int rows = img.rows;
        int cols = img.cols;
        std::vector<cv::Point> points;
        cv::findNonZero(img, points);
        if (points.size() < 2)
        {
            return std::make_pair(0, make_pair(false, k_last));
        }
        cv::Vec4f fitted_line;
        cv::fitLine(points, fitted_line, cv::DIST_L2, 0, 0.01, 0.01);
        double vx = fitted_line[0], vy = fitted_line[1], x0 = fitted_line[2], y0 = fitted_line[3];
        cv::Point2f point1, point2;
        point1.x = ((0 - y0) * vx / vy + x0);
        point1.y = 0;
        point2.x = ((rows - y0) * vx / vy + x0);
        point2.y = rows;
        cv::line(result, cv::Point(cols / 2, 0), cv::Point(cols / 2, rows), cv::Scalar(0, 0, 255), 2);
        cv::line(result, point1, point2, cv::Scalar(0, 255, 0), 2);
        // 计算拟合直线中心到图像中心的距离
        // double distance = cols / 2 - (rows * vx / vy + x0);
        double distance = cols / 2 - (point1.x + point2.x) / 2;
        // 计算斜率
        if (point2.x - point1.x == 0)
        {
            return std::make_pair(distance, std::make_pair(true, 0));
        }
        double k = -(point2.y - point1.y) / (point2.x - point1.x);
        std::cout << "k: " << k << std::endl;
        k_last = k;
        return std::make_pair(distance, std::make_pair(false, k));
    }

private:
    double k_last = 0;
    std::vector<std::vector<int>> amount;
    std::vector<std::vector<bool>> visited;
    int dfs(short x, short y, cv::Mat &img)
    {
        if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
            return 0;
        visited[x][y] = true;
        amount[x][y] = 1;
        amount[x][y] += dfs(x + 1, y, img);
        amount[x][y] += dfs(x - 1, y, img);
        amount[x][y] += dfs(x, y + 1, img);
        amount[x][y] += dfs(x, y - 1, img);
        return amount[x][y];
    }
    void dye(short x, short y, cv::Mat &img, cv::Mat &mask)
    {
        if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
            return;
        visited[x][y] = true;
        mask.at<uchar>(x, y) = 255;
        dye(x + 1, y, img, mask);
        dye(x - 1, y, img, mask);
        dye(x, y + 1, img, mask);
        dye(x, y - 1, img, mask);
    }
    void resetVisited(int rows, int cols)
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                visited[i][j] = false;
            }
        }
    }
};

class PIDController
{
    double Kp, Ki, Kd;
    double integral = 0.0;
    double previous_error = 0.0;

public:
    PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
    double P(double error)
    {
        return Kp * error;
    }
};

class AprilTagDetector
{
private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;

public:
    AprilTagDetector()
    {
        tf = tag25h9_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
    }

    ~AprilTagDetector()
    {
        apriltag_detector_destroy(td);
        tag25h9_destroy(tf);
    }

    int detect(const cv::Mat &img)
    {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};
        zarray_t *detections = apriltag_detector_detect(td, &im);
        int res = -1;
        if (zarray_size(detections) > 0)
        {
            apriltag_detection_t *det;
            zarray_get(detections, 0, &det);
            res = det->id;
        }
        apriltag_detections_destroy(detections);
        return res;
    }
};

int main()
{
    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    PIDController pid_controllerYawSlow(0.4, 0, 0);
    PIDController pid_controllerYawFast(0.6, 0, 0);
    PIDController pid_controllerY(0.001, 0, 0);
    BGRMaskGenerator BGR_mask_generator(0, 100, 0, 100, 0, 100);
    ImageProcessor ImageProcessor;
    AprilTagDetector detector;
    Camera cam(1);
    while (1)
    {
        cv::Mat frame = cam.getFrame();
        cv::imshow("frame", frame);

        int tagId = detector.detect(frame);
        if (tagId != -1)
        {
            std::cout << "tagId: " << tagId << std::endl;
        }
        else
        {
            std::cout << "tagId: " << "None" << std::endl;
        }

        // // 翻转frame
        // cv::flip(frame, frame, -1);

        // // 获取frame的宽度和高度
        // int width = frame.cols;
        // int height = frame.rows;
        // int part_height = height / 5;
        // int start_height = 2 * part_height;
        // int end_height = 4 * part_height;
        // cv::Mat roi = frame(cv::Rect(0, start_height, width, end_height - start_height));

        // cv::Mat mask0 = BGR_mask_generator.getMask(roi);
        // cv::Mat mask1 = ImageProcessor.createCustomMask(mask0);
        // cv::Mat result = roi.clone();
        // std::pair<double, std::pair<bool, double>> result_pair = ImageProcessor.liner(mask1, result);
        // cv::imshow("result", result);
        // double distance = result_pair.first;
        // double angle = 0, angle_rad = 0;
        // if (result_pair.second.first)
        // {
        //     angle_rad = M_PI / 2;
        //     angle = 90;
        // }
        // else
        // {
        //     angle_rad = atan(result_pair.second.second);
        //     angle_rad = angle_rad < 0 ? angle_rad + M_PI : angle_rad;
        //     angle = angle_rad * 180 / M_PI;
        //     angle = angle < 0 ? angle + 180 : angle;
        // }
        // // 输出距离和角度
        // std::cout << "distance: " << distance << " angle: " << angle << std::endl;
        // double angle_goal_rad = M_PI / 2.0;
        // double angle_error = angle_rad - angle_goal_rad;
        // std::cout << "angle_error: " << angle_error << std::endl;
        // PIDController *pid_controllerYaw = (abs(angle_error) < M_PI / 6 ? &pid_controllerYawSlow : &pid_controllerYawFast);
        // double yaw_speed = pid_controllerYaw->P(angle_error);
        // std::cout << "yaw_speed: " << yaw_speed << std::endl;
        // double y_speed = pid_controllerY.P(distance);
        // std::cout << "y_speed: " << y_speed << std::endl;
        // custom.setVelocity(0.1, y_speed, yaw_speed);
        // // custom.setVelocity(0, y_speed, 0);
        // // custom.setVelocity(0.1, 0, 0);
        // // custom.setVelocity(0, 0, 0.1);

        cv::waitKey(1);
    }
}
