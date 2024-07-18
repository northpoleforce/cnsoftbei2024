#include <DogVision.h>
#include <FrontCamera.h>
#include <thread>
#define DOG_MOTION_VISION
// #define DOG_VISION_ONLY

#ifdef AGENT_TOOL
int main(int argc, char *argv[]) {
    // 主函数的代码逻辑
    int sportTime(0);
    if (argc >= 2) {
        sportTime = std::atoi(argv[1]);  // atoi用来将字符串转为数字
    }
    DogMotion myplan(HIGHLEVEL);
    myplan.setPlanTime(sportTime);
    std::thread dogGoPath(&DogMotion::Run, &myplan);
    dogGoPath.join();
    return 0;

    return 0;
}
#endif


#ifdef DOG_All_EXAMPLE
int main() {
//    DogSport sport(HIGHLEVEL);
    DogMotion myplan(HIGHLEVEL);
    FrontCamera frontCamera(&myplan);
    DogVision detector("../config/detectMid.yaml", &myplan);
    std::thread dogVisionThread(&DogVision::Start, &detector);
    std::thread dogDetectObstacle(&FrontCamera::Start, &frontCamera);
    std::thread dogGoPath(&DogMotion::Run, &myplan);
    dogVisionThread.join();
    dogGoPath.join();
    dogDetectObstacle.join();
    return 0;
}
#endif

#ifdef  DOG_MOTION_VISION
int main() {
    DogMotion myplan(HIGHLEVEL); // 运动模式：高层控制
    DogVision detector("../config/detectMid.yaml", &myplan); // 视觉
    std::thread dogVisionThread(&DogVision::Start, &detector); // 开启视觉线程
    std::thread dogGoPath(&DogMotion::Run, &myplan);    // 开启运动线程
    dogVisionThread.join(); // 等待视觉线程结束
    dogGoPath.join(); // 等待运动线程结束
    // 两线程并行执行，互不干扰，等待两线程结束后，主线程结束
    return 0;
}
#endif

#ifdef DOG_VISION_ONLY
int main() {
    DogMotion myplan(HIGHLEVEL);
    printf("myplan ok\n");
    // FrontCamera frontCamera(&myplan);
    DogVision detector("../config/detectMid.yaml", &myplan);
    printf("detector ok\n");
    std::thread dogVisionThread(&DogVision::Start, &detector);
    dogVisionThread.join();
    return 0;
}
#endif


#ifdef  DOG_MOTION_ONLY
int main() {
    DogMotion myplan(HIGHLEVEL);
    myplan.planTimes = 5; // now RobotControl's content is GoForward, so it will go forword 5 s approximately
    myplan.Run();
    return 0;
}
#endif


#ifdef DOG_FRONT_ONLY
int main() {
    DogVision detector("../config/detectMid.yaml", &myplan);
    std::thread dogVisionThread(&DogVision::Start, &detector);
    dogVisionThread.join();
    return 0;


#endif

#ifdef VIDEO_TEST
int main(int argc, char** argv) {
    VideoCapture v("/Users/oplin/CLionProjects/opencv/robocom_vision/test_image/test1.mp4");
    Mat src;
    while (true) {
        if (!v.isOpened()) {
            break;
        }
        v.read(src);
        if (src.empty()) {
            break;
        }
//            camera_warrper->read_frame_rgb(src);
        DogVision detector("../config/detectMid.yaml");
        detector.createMyTrackbar();
        detector.processImage(src);
    }
    return 0;
}
#endif

