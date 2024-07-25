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

    cam2.reopen();
    PIDController pidXY(0.002, 0, 0);
    PIDController pidYaw(0.02, 0, 0);
    while (true)
    {
        cv::Mat frame = cam2.getFrame();
        int degreeVertical;
        cv::Point cornerMidPoint, midPointFrame(frame.cols / 2, frame.rows / 2);
        lineProcessor.cornerFit(frame, cornerMidPoint, degreeVertical);
        std::cout << "cornerMidPoint: " << cornerMidPoint << " degreeVertical: " << degreeVertical << std::endl;
        if (lessTolerance(midPointFrame.x, cornerMidPoint.x, 10) && lessTolerance(90, degreeVertical, 5))
        {
            custom.cmdReset();
            break;
        }
        custom.setVelocity(-min(pidXY.P(midPointFrame.y, cornerMidPoint.y), 0.1),
                           -min(pidXY.P(midPointFrame.x, cornerMidPoint.x), 0.1),
                           min(pidYaw.P(90, degreeVertical), 0.1));
    }
    custom.cmdReset();
    std::cout << taskID << " corner finished" << std::endl;
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

    // 移出启动区
    {
        custom.moveLeft(0.7, 0.2);
    }
    // 第一个转角：位姿矫正
    cornerCorrect(1, cam2, lineProcessor, custom);
    // 前进至第二个转角
    {
        // custom.moveForward(1.2, 0.2);
        sleep(1);
        custom.moveForward(1.3, 0.2);
    }
    // 第二个转角：位姿矫正
    cornerCorrect(2, cam2, lineProcessor, custom);
    // 后退至第三个转角
    {
        sleep(2);
        custom.moveForward(-1.2, 0.2);
    }
    // 第三个转角：位姿矫正
    cornerCorrect(3, cam2, lineProcessor, custom);
    // 左移至第四个转角
    {
        sleep(2);
        custom.moveLeft(1.1, 0.2);
        // custom.moveForward(-0.1, 0.2);
    }
    // 第四个转角：位姿矫正
    cornerCorrect(4, cam2, lineProcessor, custom);
    // 前进至第五个转角
    {
        sleep(2);
        custom.moveForward(1, 0.2);
    }
    // 第五个转角：位姿矫正
    cornerCorrect(5, cam2, lineProcessor, custom);
    {
        cam2.reopen();
        while (1)
        {
            cv::Mat frame = cam2.getFrame();
            cv::imshow("frame", frame);
            cv::waitKey(1);
        }
    }

    return 0;
}