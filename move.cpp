/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <thread>
#include <cmath>

#include "move.h"

void Custom::UDPRecv()
{
  udp.Recv();
}
void Custom::UDPSend()
{
  udp.Send();
}
void Custom::RobotControl()
{
  udp.GetRecv(state);
  udp.SetSend(cmd);
}

void Custom::cmdReset()
{
  cmd.mode = 0;
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;
}



double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
void Custom::turnLeft_Degree(float degree, float speed = 30)
{
  degree = degreesToRadians(degree);
  speed = degreesToRadians(speed);
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.yawSpeed = speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(degree / speed));
  cmdReset();
}
void Custom::turnLeft90()
{
  // 120 30, 约90度
  Custom::turnLeft_Degree(120, 30);
}
void Custom::turnRight_Degree(float degree, float speed = 30)
{
  // 120 30, 约90度
  degree = degreesToRadians(degree);
  speed = degreesToRadians(speed);
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.yawSpeed = -speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(degree / speed));
  cmdReset();
}


void Custom::forwardWalk_m(float distance, float speed = 0.5)
{
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[0] = speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(distance / speed));
  cmdReset();
}
void Custom::leftWalk_m(float distance, float speed = 0.5)
{
  // 速度慢：怎么会边走边后退？？
  // 1m/s：非常勉强，勉强能用
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[1] = speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(distance / speed));
  cmdReset();
}
void Custom::rightWalk_m(float distance, float speed = 0.5)
{
  // 2 0.8, 可走约2m，直行方向基本无偏差
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[1] = -speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(distance / speed));
  cmdReset();
}
void Custom::leftmove_test(int gType, float distance, float speed)
{
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = gType;
  cmd.velocity[1] = speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(distance / speed));
  cmdReset();
}

void Custom::putLeft()
{
  cmdReset();
  cmd.mode = 1;
  cmd.euler[1] = -0.75;
  sleep(1);
  cmd.euler[1] = 0.0;
  sleep(1);
  cmd.bodyHeight = -0.2;
  sleep(1);
  cmd.euler[0] = -0.75;
  sleep(1);
  cmd.euler[0] = 0;
  sleep(1);
  cmd.bodyHeight = 0;
  sleep(1);
}
void Custom::putRight()
{
  cmdReset();
  cmd.mode = 1;
  cmd.euler[1] = +0.75;
  sleep(1);
  cmd.euler[1] = 0.0;
  sleep(1);
  cmd.bodyHeight = -0.2;
  sleep(1);
  cmd.euler[0] = +0.75;
  sleep(1);
  cmd.euler[0] = 0;
  sleep(1);
  cmd.bodyHeight = 0;
  sleep(1);
}
void Custom::standDown()
{
  cmdReset();
  cmd.mode = 1;
  sleep(1);
  cmd.mode = 6;
  sleep(1);
  cmd.mode = 5;
  sleep(1);
  cmd.mode = 7;
  sleep(1);
}
void Custom::standUp()
{
  cmdReset();
  cmd.mode = 5;
  sleep(1);
  cmd.mode = 6;
  sleep(1);
  cmd.mode = 1;
  sleep(1);
}
void Custom::setVelocity(float vx, float vy, float vr)
{
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[0] = vx;
  cmd.velocity[1] = vy;
  cmd.yawSpeed = vr;
  std::cout << "vx: " << vx << " vy: " << vy << " vr: " << vr << std::endl;
}

void Custom::showIMU()
{
  IMU imu = state.imu;
  std::cout << "IMU: " << imu.accelerometer[0] << " " << imu.accelerometer[1] << " " << imu.accelerometer[2] << " "
            << imu.gyroscope[0] << " " << imu.gyroscope[1] << " " << imu.gyroscope[2] << " "
            << imu.rpy[0] << " " << imu.rpy[1] << " " << imu.rpy[2] << std::endl;
}

void Custom::Start()
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;

  Custom custom(HIGHLEVEL);
  LoopFunc loop_control("control_loop", dt, boost::bind(&Custom::RobotControl, this));
  LoopFunc loop_udpSend("udp_send", dt, 3, boost::bind(&Custom::UDPSend, this));
  LoopFunc loop_udpRecv("udp_recv", dt, 3, boost::bind(&Custom::UDPRecv, this));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };
}
