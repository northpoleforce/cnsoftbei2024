#include "move.h"
#include "Camera.h"
#include "MapImageProcessing.h"

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
    const int xtolerance = 20, ytolerance = 20, yawtolerance = 5;
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

    // 第3个动物识别后对位
    cornerCorrect(8, cam2, lineProcessor, custom);
    // 右转90度
    custom.rotateLeft(-90, 30);
    custom.forwardWalkNew(0.1, 0.2);

    // 避障开始对位
    cornerCorrect(9, cam2, lineProcessor, custom);
    // 清理障碍
    custom.setVelocity(0, -0.5, 0), sleep(1), custom.cmdReset();
    custom.setVelocity(0.5, 0, 0), sleep(1), custom.cmdReset();
    sleep(1);
    custom.setVelocity(0, 1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(700)));
    custom.cmdReset();
    sleep(1);
    custom.setVelocity(-0.2, 0, 0), sleep(1), custom.cmdReset();

    // 避障区起点再对位
    cornerCorrect(10, cam2, lineProcessor, custom);
    custom.forwardWalkNew(2.2, 0.2);                // 移动至避障区终点
    custom.setVelocity(0, -0.2, 0), sleep(1), custom.cmdReset();
    cornerCorrect(11, cam2, lineProcessor, custom); // 避障区终点对位

    // 避障结束对位
    cornerCorrect(12, cam2, lineProcessor, custom);
    custom.rotateLeft(-90, 30);                                  // 右转90度
    custom.setVelocity(-0.2, 0, 0), sleep(1), custom.cmdReset(); // 移动至大概对位位置
    custom.setVelocity(0, 0.1, 0), sleep(1), custom.cmdReset();
    // 终点前对位
    cornerCorrect(12, cam2, lineProcessor, custom);
    // 移动至终点
    custom.setVelocity(0, 0.28, 0), sleep(4), custom.cmdReset();
    custom.setVelocity(0.1, 0, 0), sleep(2), custom.cmdReset();

    return 0;
}