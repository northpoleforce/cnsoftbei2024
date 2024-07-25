#include "move.h"
#include "Camera.h"
#include "MapImageProcessing.h"
#include "Detect_TensorRT.hpp"

// 目标全局变量
bool isMonkeyDetected = false;
bool isPandaDetected = false;
bool isWolfDetected = false;
const std::vector<std::string> CLASS_NAMES = {
        "monkey",
        "panda",
        "wolf"
};
const std::vector<std::vector<unsigned int>> COLORS = {
        {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238}
};
std::string hostname = "192.168.123.13";
std::string username = "unitree";
std::string password = "123";  // 密码

// 容忍度判断
bool lessTolerance(float target, float value, float tolerance)
{
    if (abs(target - value) < tolerance)
        return true;
    return false;
}

void cornerCorrect(int taskID, Camera &cam2, LineProcessor &lineProcessor, Custom &custom)
{
    PIDController pidXY(0.001, 0, 0);
    PIDController pidYaw(0.05, 0, 0);
    cam2.reopen();
    const int xtolerance = 10, ytolerance = 10, yawtolerance = 5;
    const int xoffset = 30, yoffset = 30;
    while (true)
    {
        cv::Mat frame = cam2.getFrame();
        cv::Point cornerMidPoint, targetPoint(frame.cols / 2 - xoffset, frame.rows / 2 - yoffset);
        int degreeVertical;
        lineProcessor.cornerFit(frame, cornerMidPoint, degreeVertical);
        std::cout << "cornerMidPoint: " << cornerMidPoint << " degreeVertical: " << degreeVertical << std::endl;
        if (lessTolerance(targetPoint.x, cornerMidPoint.x, xtolerance) &&
            lessTolerance(targetPoint.y, cornerMidPoint.y, ytolerance) &&
            lessTolerance(90, degreeVertical, yawtolerance))
        {
            custom.cmdReset();
            break;
        }
        double speedX = -pidXY.P(targetPoint.y, cornerMidPoint.y);
        speedX = min(speedX, 0.2), speedX = max(speedX, -0.2);
        double speedY = -pidXY.P(targetPoint.x, cornerMidPoint.x);
        speedY = min(speedY, 0.2), speedY = max(speedY, -0.2);
        double speedYaw = pidYaw.P(90, degreeVertical);
        speedYaw = min(speedYaw, 0.2), speedYaw = max(speedYaw, -0.2);
        custom.setVelocity(speedX, speedY, speedYaw);
    }
    std::cout << "\033[1;31m" << taskID << " corner finished" << "\033[0m" << std::endl;
}

// 用于播喇叭
int executeRemoteCommand(const std::string& hostname, const std::string& username, const std::string& password, const std::string& command) {
    std::string sshCommand = "sshpass -p '" + password + "' ssh " + username + "@" + hostname + " \"" + command + "\"";
    int result = system(sshCommand.c_str());
    return result;
}

void detectAnimals(YOLOv8 *yolov8, Custom &custom) {
    cv::VideoCapture cap(0);
    cv::Size            size = cv::Size{640, 640};
    std::vector<Object> objs;
    cv::Mat img, res; //要读图像就是 cap >> img;
    bool isDetected = false;
    while (!isDetected) {
        cap >> img;
        if (img.empty()) {
            cout << "no img!!!" << endl;
            continue;
        }
        // infer model
        objs.clear();
        yolov8->copy_from_Mat(img, size);
        yolov8->infer();
        yolov8->postprocess(objs);
        std::string obj_cls;
        obj_cls = yolov8->draw_objects(img, res, objs, CLASS_NAMES, COLORS);
//        cout << "Detect : " << obj_cls << ' ';
        // cv::imshow("result", res);
        // cv::waitKey(1);
        if (obj_cls == "monkey" && !isMonkeyDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/monkey.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.lower();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
            isMonkeyDetected = true;
            isDetected = true;
        }

        if (obj_cls == "panda" && !isPandaDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/panda.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.rise();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;

            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
            isPandaDetected = true;
            isDetected = true;
        }

        if (obj_cls == "wolf" && !isWolfDetected) {
            std::string command = "amixer -c 2 set Speaker 26 && aplay -D plughw:2,0 /home/unitree/Desktop/24SoftCup/wolf.wav";
            int result = executeRemoteCommand(hostname, username, password, command);
            custom.warning();
            if (result == 0) {
                std::cout << "Command executed successfully" << std::endl;
            } else {
                std::cout << "Failed to execute command" << std::endl;
            }
            isWolfDetected = true;
            isDetected = true;
        }
    }
}

int main()
{
    // 启动运动控制通信
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    // 等待通信建立
    sleep(1);

    // 图像处理
    LineProcessor lineProcessor;
    // 开摄像头
    Camera cam2(2);

    //TODO 以下是目标检测需要用到的变量
    string engine_file_path = "/home/unitree/Documents/24SoftCupCodes/24SoftCupFOlder/detectModels/train8/weights/jetson_new.engine";
    cv::Size            size = cv::Size{640, 640};
    std::vector<Object> objs;
    auto yolov8 = new YOLOv8(engine_file_path);
    yolov8->make_pipe(true);

    // 移出启动区
    custom.moveLeft(0.7, 0.2);
    // 第一个转角：位姿矫正
    cornerCorrect(1, cam2, lineProcessor, custom);

    // 前进至第二个转角
    sleep(1), custom.moveForward(1.4, 0.2);
    // 第二个转角：位姿矫正
    cornerCorrect(2, cam2, lineProcessor, custom);

    // 进入第一个巡检, 检测第一个动物
    detectAnimals(yolov8, custom);

    // 后退至第三个转角
    sleep(1), custom.moveForward(-1.2, 0.2);
    // 第三个转角：位姿矫正
    cornerCorrect(3, cam2, lineProcessor, custom);
    
    // 左移至第四个转角
    sleep(1), custom.moveLeft(1.1, 0.2);
    // 第四个转角：位姿矫正
    cornerCorrect(4, cam2, lineProcessor, custom);
    
    // 前进至第五个转角
    sleep(1), custom.moveForward(1.2, 0.2);
    // 第五个转角：位姿矫正
    cornerCorrect(5, cam2, lineProcessor, custom);

    // 进入第二个巡检, 检测第二个动物
    detectAnimals(yolov8, custom);

    // 左转90度
    custom.rotateLeft(90, 30);
    custom.leftWalkNew(-0.2, 0.3), custom.forwardWalkNew(0.2, 0.3);

    // 第6个转角：位姿矫正
    cornerCorrect(6, cam2, lineProcessor, custom);
    // 前进至第7个转角
    sleep(1), custom.forwardWalkNew(1.2, 0.2, 3);
    // 右转90度
    custom.rotateLeft(-90, 30);
    // custom.forwardWalkNew(-0.3, 0.3);
    custom.moveForward(-0.3, 0.3);

    // 第7个转角：位姿矫正
    cornerCorrect(7, cam2, lineProcessor, custom);
    // custom.forwardWalkNew(1.8, 0.2);
    // custom.leftWalkNew(-0.2, 0.2);
    custom.moveForward(1.9, 0.2);
    custom.moveLeft(-0.2, 0.2);
    cornerCorrect(8, cam2, lineProcessor, custom);

    // 进入第三个巡检, 检测第三个动物
    detectAnimals(yolov8, custom);

    // custom.moveForward(1.2, 0.2);
    // {
    //     cam2.reopen();
    //     while (1)
    //     {
    //         cv::Mat frame = cam2.getFrame();
    //         cv::imshow("frame", frame);
    //         cv::waitKey(1);
    //     }
    // }

    return 0;
}