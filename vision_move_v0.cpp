#include "move.h"
#include "Camera.h"
#include "MapImageProcessing.h"

int main()
{
    // 启动运动控制通信
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    // 等待通信建立
    sleep(1);
    // PID控制器
    PIDController pidXY(0.003, 0, 0);
    PIDController pidYaw(0.02, 0, 0);

    // 图像处理
    LineProcessor lineProcessor;
    // 建立摄像头通信
    Camera cam2(2);

    custom.moveLeft(0.6, 0.2);
    sleep(1);
    cam2.reopen();
    while (true)
    {
        cv::Mat frame = cam2.getFrame();
        cv::Point midPointLine, midPointFrame(frame.cols / 2, frame.rows / 2);
        int degreeK;
        lineProcessor.lineFit(frame, midPointLine, degreeK);
        std::cout << "midPointLine: " << midPointLine << " degreeK: " << degreeK << std::endl;
        custom.setVelocity(0, -pidXY.P(midPointFrame.x, midPointLine.x), pidYaw.P(90, degreeK));
    }


    return 0;
}