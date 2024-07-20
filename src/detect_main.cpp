#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <unistd.h>

#include "move.h"

#include "chrono"
#include "Detect_TensorRT.hpp"
#include <cstdlib>
struct Tasks
{
    bool task_have[5], task_done[5];
    Tasks()
    {
        memset(task_have, false, sizeof(task_have));
        memset(task_done, false, sizeof(task_done));
        task_have[0] = true;
    }
};
Tasks tasks;

class ControlRect
{
public:
    ControlRect() {}
    int width = 0, height = 0; // 图像的大小
    int left = 0, top = 0;     // 控制矩形的左上角
    int right = 0, bottom = 0; // 控制矩形的右下角
};
class ControlParams
{
public:
    ControlParams(int atc, double vX,
                  double Y_Kp, double Y_Ki, double Y_Kd,
                  double R_Kp, double R_Ki, double R_Kd) : apriltagCOUNT_DOWN(atc),
                                                           vX(vX),
                                                           Y_Kp(Y_Kp), Y_Ki(Y_Ki), Y_Kd(Y_Kd),
                                                           R_Kp(R_Kp), R_Ki(R_Ki), R_Kd(R_Kd)
    {
    }
    int apriltagCOUNT_DOWN = 10;
    double vX = 0.0;
    double Y_Kp = 0.0, Y_Ki = 0.0, Y_Kd = 0.0;
    double R_Kp = 0.0, R_Ki = 0.0, R_Kd = 0.0;
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

ControlRect controlRect;
CheckPoints cp;

cv::Mat loadImage(const std::string &imagePath)
{
    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cout << "Could not read the image: " << imagePath << std::endl;
        exit(1);
    }
    // 显示图片并返回
    cv::imshow("Image", img);
    return img;
}
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
    int width = hsv.cols / 4;
    int height = hsv.rows / 4;
    controlRect.left = (hsv.cols - width) / 2;
    controlRect.top = (hsv.rows - height) / 2;
    controlRect.right = controlRect.left + width;
    controlRect.bottom = controlRect.top + height * 2;
    // 获取控制关键（下巴）
    // controlRect.top = hsv.rows * (1 - 1 / 20.0);
    // controlRect.left = 0;
    // controlRect.bottom = hsv.rows;
    // controlRect.right = hsv.cols;
    // 显示掩膜并返回
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
        std::string udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
        std::cout << udp_send_integrated_pipe_0 << std::endl;
        cap.open(udp_send_integrated_pipe_0);
    }
    ~Camera()
    {
        cv::destroyAllWindows();
        cap.release();
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
        // cv::imshow("original", frame);
        // if (cv::waitKey(2) == 'q')
        // {
        //     // cv::destroyAllWindows();
        //     // exit(0);
        // }
        return frame;
    }

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
};

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

// #define RELEASE
// #define DEBUG_APRILTAG
// #define TENSOR
#define CAMERA
// #define POSE
//#define VISION
// #define DEBUG_PUT_DOWEN

#ifdef DEBUG_PUT_DOWEN
int main()
{
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);

    while (1)
    {
        custom.putLeft();
        custom.putRight();
    }

    return 0;
}
#endif

#ifdef VISION
int apriltagDetector(const cv::Mat &img)
{
    // Initialize AprilTag detector
    apriltag_family_t *tf = tagStandard41h12_create();    // 一个AprilTag标签族的实例
    apriltag_detector_t *td = apriltag_detector_create(); // 一个AprilTag检测器的实例
    apriltag_detector_add_family(td, tf);                 // 标签族添加到检测器中

    // apriltag只能处理灰度图像
    cv::Mat gray;
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
    tagStandard41h12_destroy(tf);
    return res;
}
int lookingMarker(const cv::Mat &img)
{
    int marker_id = apriltagDetector(img);
    if (marker_id == -1) return -1;
    std::cout << "\033[1;31mDetected tag ID: " << marker_id << "\033[0m" << std::endl;
    if (cp.time_marker[marker_id] == -1)
        cp.time_marker[marker_id] = std::time(nullptr);
    return marker_id;
}

void readTasks()
{
    int task1ID = -1;
    std::cout << "\033[1;33mWaiting for 1st task:\033[0m " << std::endl;
    while (1)
    {
        Camera cam(1);
        while (1)
        {
            cv::Mat img = cam.getFrame();
            task1ID = apriltagDetector(img);
            if (task1ID != -1)
                break;
        }
        if (task1ID != -1)
            break;
    }
    std::cout << "\033[1;32m1st task detected: " << task1ID << "\033[0m" << std::endl;
    std::cout << "\033[1;33mpress ENTER for 2nd task reading...\033[0m " << std::endl;
    std::cin.get();
    std::cout << "\033[1;33mWaiting for 2nd task:\033[0m " << std::endl;
    int task2ID = -1;
    while (1)
    {
        Camera cam(1);
        while (1)
        {
            cv::Mat img = cam.getFrame();
            task2ID = apriltagDetector(img);
            if (task2ID != -1)
                break;
        }
        if (task2ID != -1)
            break;
    }
    std::cout << "\033[1;32m2ndt task detected: " << task2ID << "\033[0m" << std::endl;
    tasks.task_have[task1ID] = true;
    tasks.task_have[task2ID] = true;
    std::cout << "\033[1;31m1st task: " << task1ID << "\n2st task: " << task2ID << "\033[0m" << std::endl;
}
void vison()
{
    Camera cam(5);
    while (1)
    {
        cv::Mat img = cam.getFrame();
        apriltagDetector(img);

        // cv::Mat mask = getMask(img, 10, 30, 50, 255, 0, 255);
        // showHighlightMask(img, mask, 0, 255, 0);

        // if (cv::waitKey(2) == 'q')
        //     break;
    }
}
#endif

#ifdef DEBUG_AUTO_CRUISE
// bool cp_start_first()
// {
//     return cp.time_marker[0][0] > -1;
// }

bool cp_markerI_first(int i)
{
    return cp.time_marker[i] > -1;
}
bool cp_markerI_out_s(int i, int s)
{
    return std::difftime(std::time(nullptr), cp.time_marker[i]) >= s;
}
bool taskI_check(int i)
{
    return tasks.task_have[i] && !tasks.task_done[i];
}

// // 最安全的参数
// ControlParams params(
//     10,
//     0.1,
//     0.00002, 0, 0,
//     0.0001, 0, 0);
// 稍微快点有问题的参数
ControlParams params(
        10,
        0.12,
        0.00002, 0, 0,
        0.00012, 0, 0);
PIDController pidY(params.Y_Kp, params.Y_Ki, params.Y_Kd);
PIDController pidR(params.R_Kp, params.R_Ki, params.R_Kd);

void walking(const cv::Mat &img, Custom &custom)
{
    cv::Rect rect(controlRect.left, controlRect.top, controlRect.right - controlRect.left, controlRect.bottom - controlRect.top);

    cv::Mat mask = getMask(img, 10, 40, 50, 255, 0, 255);
    showHighlightMask(img, mask, 0, 255, 0);

    auto [leftCount, rightCount] = countWhitePixelsOnSides(mask, rect);
    auto err = leftCount - rightCount;
    std::cout << "Error: " << err << std::endl;
    float Youtput = pidY.control(0, err);
    float Routput = pidR.control(0, err);
    std::cout << "Youtput: " << Youtput << std::endl;
    std::cout << "Routput: " << Routput << std::endl;
    custom.setVelocity(params.vX, Youtput, Routput);

    if (cv::waitKey(2) == 'q')
    {
        exit(0);
    }
}
#endif

int executeRemoteCommand(const std::string& hostname, const std::string& username, const std::string& password, const std::string& command) {
    std::string sshCommand = "sshpass -p '" + password + "' ssh " + username + "@" + hostname + " \"" + command + "\"";
    int result = system(sshCommand.c_str());
    return result;
}

#ifdef TENSOR
int main()
{
    Camera cam(1);

    // Init CUDA and TensorRT
    string engine_file_path = "/home/unitree/Documents/24SoftCupCodes/24SoftCupFOlder/detectModels/train8/weights/jetson_new.engine";
    cv::Size            size = cv::Size{640, 640};
    std::vector<Object> objs;
    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
    const std::vector<std::string> CLASS_NAMES = {
            "monkey",
            "panda",
            "wolf"
    };
    const std::vector<std::vector<unsigned int>> COLORS = {
            {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238}
    };
    // voice
    std::string hostname = "192.168.123.13";
    std::string username = "unitree";
    std::string password = "123";  // 明文密码
    bool isMonkeyDetected = 0;
    bool isPandaDetected = 0;
    bool isWolfDetected = 0;

    //motion control
    Custom custom(HIGHLEVEL);
   std::thread MoveControl(&Custom::Start, &custom);

    while (1)
    {
        cv::Mat img = cam.getFrame(), res;
        if (img.empty()) {
            cout << "no img!!!" << endl;
            continue;
        }
        // infer model
        objs.clear();
        yolov8->copy_from_Mat(img, size);
        auto start = std::chrono::system_clock::now();
        yolov8->infer();
        auto end = std::chrono::system_clock::now();
        yolov8->postprocess(objs);
        std::string obj_cls;
        obj_cls = yolov8->draw_objects(img, res, objs, CLASS_NAMES, COLORS);
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        cout << "Detect : " << obj_cls << ' ';
        printf("Infer cost %2.4lf ms\n", tc);
        // cv::imshow("result", res);
        if (obj_cls == "monkey" && !isMonkeyDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/monkey.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.lower();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isMonkeyDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }

        if (obj_cls == "panda" && !isPandaDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/panda.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.rise();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isPandaDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }

        if (obj_cls == "wolf" && !isWolfDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/wolf.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.warning();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isWolfDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }


        if (cv::waitKey(2) == 'q')
            break;
    }
    cv::destroyAllWindows();
    delete yolov8;
    return 0;
}
#endif

#ifdef POSE
int main()
{
   Custom custom(HIGHLEVEL);
   std::thread MoveControl(&Custom::Start, &custom);
   custom.warning();
   return 0;

}
#endif

#ifdef CAMERA
int main()
{
    cv::VideoCapture cap(0);

    // Init CUDA and TensorRT
    string engine_file_path = "/home/unitree/Documents/24SoftCupCodes/24SoftCupFOlder/detectModels/train8/weights/jetson_new.engine";
    cv::Size            size = cv::Size{640, 640};
    std::vector<Object> objs;
    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
    const std::vector<std::string> CLASS_NAMES = {
            "monkey",
            "panda",
            "wolf"
    };
    const std::vector<std::vector<unsigned int>> COLORS = {
            {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238}
    };
    // voice
    std::string hostname = "192.168.123.13";
    std::string username = "unitree";
    std::string password = "123";  // 明文密码
    bool isMonkeyDetected = 0;
    bool isPandaDetected = 0;
    bool isWolfDetected = 0;

    //motion control
    Custom custom(HIGHLEVEL);
   std::thread MoveControl(&Custom::Start, &custom);
    sleep(2);
    while (1)
    {
        cv::Mat img, res;
        cap >> img;
        if (img.empty()) {
            cout << "no img!!!" << endl;
            continue;
        }
        // infer model
        objs.clear();
        yolov8->copy_from_Mat(img, size);
        auto start = std::chrono::system_clock::now();
        yolov8->infer();
        auto end = std::chrono::system_clock::now();
        yolov8->postprocess(objs);
        std::string obj_cls;
        obj_cls = yolov8->draw_objects(img, res, objs, CLASS_NAMES, COLORS);
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        cout << "Detect : " << obj_cls << ' ';
        printf("Infer cost %2.4lf ms\n", tc);
        // cv::imshow("result", res);
        if (obj_cls == "monkey" && !isMonkeyDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/monkey.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.lower();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isMonkeyDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }

        if (obj_cls == "panda" && !isPandaDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/panda.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.rise();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isPandaDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }

        if (obj_cls == "wolf" && !isWolfDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/wolf.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.warning();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
                isWolfDetected = 1;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
        }


        if (cv::waitKey(2) == 'q')
            break;
    }
    cap.release();
    cv::destroyAllWindows();
    delete yolov8;
    return 0;
}
#endif