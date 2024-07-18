#pragma once
#ifndef GO1_CONTROL_DogMotion_H
#define GO1_CONTROL_DogMotion_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
using namespace UNITREE_LEGGED_SDK;
class DogMotion
{
public:
    DogMotion(uint8_t level);
    // Unitree official function for dog's movement control
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void Run();

    // My function to control go1
    void GoForward(int time, bool useIMU, int deflection, bool useVision);
    void TurnRight(int time,double v0, bool useVision);
    void TurnLeft(int time,double v0, bool useVision);
    void LeftTranslation(int time,bool useVision);
    void RightTranslation(int time,bool useVision);
    void RightCircle(int time,double v0,double yaws);
    void LeftCircle(int time,double v0,double yaws);
    void Lean_Forward();
    void Lean_Backward();
    void Stop();
    void setPlanTime(int seconds);

    // official variables to initialize
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    void initState(); // init all param
    void visionControl(); // use vision control
    // vision environment variable
    bool isVision = false;
    bool midShiftRight = false;
    bool midShiftLeft = false;
    bool yellowRight = false;
    bool yellowLeft = false;
    bool greenRight = false;
    bool greenLeft = false;

    // Motion
    int flag1=0,flag2=0,flag3=1,flag4=1;

    // Yaw
    void getInitIMU(int time);
    void yawCorrect(int deflection);
    float initYaw=0.0f;
    bool isFirst=false;

    // Plan Motion
    int planTimes=0;
};




#endif //GO1_CONTROL_DogMotion_H
