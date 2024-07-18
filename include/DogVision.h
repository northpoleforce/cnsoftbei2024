#pragma once
#ifndef GO1_CONTROL_DOGVISION_H
#define GO1_CONTROL_DOGVISION_H
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <algorithm>
#include "DogMotion.h"

using namespace cv;
using namespace std;

// 声明一个DogVision类，这个类包含图像处理所需要的所有参数，
// 并且提供了一个processImage函数用于处理图像。
class DogVision {
private:
    UnitreeCamera cam;
    int cannyThreshold, cannyThresholdMax, houghThreshold, houghMinLineLength, houghMaxLineGap;
    int yellowHLower, yellowSLower, yellowVLower, yellowHUpper, yellowSUpper, yellowVUpper;
    int lowerAngel, upperAngel;

public:
    DogVision();
    DogVision(const std::string& configFile, DogMotion* myPlan);
    // 通过读取一个配置文件来构造DogVision对象。
    // 在构造对象时，将会从配置文件中读取参数并初始化它们。
    void processImage(cv::Mat& src);
    void detectYellow(cv::Mat& src);
    void detect(cv::Mat src);
    void Start();
//    bool isTime();
    DogMotion* dogMotion;
};



#endif //GO1_CONTROL_DOGVISION_H
