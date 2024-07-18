#ifndef MOV_H
#define MOV_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : 
    safe(LeggedType::Go1),
    udp(level, 8090, "192.168.123.161", 8082)
  {
    udp.InitCmdData(cmd);
  }

  void UDPRecv();
  void UDPSend();
  void RobotControl();
  
  void cmdReset();
  
  void turnLeft_Degree(float degree, float speed);
  void turnLeft90();
  void turnRight_Degree(float degree, float speed);

  void forwardWalk_m(float distance, float speed);
  void leftWalk_m(float distance, float speed);
  void rightWalk_m(float distance, float speed);
  void leftmove_test(int gType, float distance, float speed);

  void putLeft();
  void putRight();
  void setVelocity(float vx, float vy, float vr);
  void standDown();
  void standUp();

  void showIMU();

  void Start();

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
  int motiontime = 0;
  // float dt = 0.002; // 0.001~0.01
  float dt = 0.01; // 0.001~0.01

  float vx = 0;
  float vy = 0;
  float vr = 0;
};

#endif