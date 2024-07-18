#ifndef FRONTCAMERA_H
#define FRONTCAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>
#include <DogMotion.h>

using namespace cv;
using namespace std;

class FrontCamera
{
private:
    VideoCapture cap;
    int minH,minS,minV,maxH,maxS,maxV;
public:
    FrontCamera(DogMotion* myPath);
    void Start();
    DogMotion* dogMotion;
};

#endif

