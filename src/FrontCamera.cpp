#include "FrontCamera.h"
using namespace std;
using namespace cv;

FrontCamera::FrontCamera(DogMotion* myPlan) : cap(string("udpsrc address=192.168.123.15 port=9201 ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink"))
{
    this->minH = 60; this->minS = 100; this->minV = 100;
    this->maxH = 90; this->maxS = 190; this->maxV = 150;
    dogMotion = myPlan;
}

void FrontCamera::Start()
{
    if(!cap.isOpened())
    {
        std::cout << "can not open the camera in class(FrontCamera)" << std::endl;
        return;
    }
    cv::Mat frame;
    while(1)
    {    
        cap >> frame;
        if(frame.empty())
            break;
        flip(frame, frame, -1);
        frame = frame(cv::Rect(frame.cols / 7, 0, 5 * frame.cols / 7, frame.rows));
        frame = frame(cv::Rect(0, frame.rows / 7, frame.cols, 5 * frame.rows / 7));
        resize(frame, frame, Size(398, 400));
        // TODO below are what you want to detect
        // detectObstacle(frame);
        cv::waitKey(20);
    }
    cap.release();//释放资源
}


