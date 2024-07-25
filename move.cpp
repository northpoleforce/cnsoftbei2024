/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Dense>

#include "move.h"

PIDController::PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
double PIDController::P(double setpoint, double measured_value)
{
  double error = setpoint - measured_value;
  return Kp * error;
}

KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimationError)
{
  A << 1, 1,
      0, 1;    // 状态转移矩阵
  B << 0.5, 1; // 控制输入矩阵
  H << 1, 0;   // 测量矩阵
  Q << processNoise, 0,
      0, processNoise;   // 过程噪声协方差矩阵
  R << measurementNoise; // 测量噪声协方差矩阵
  P << estimationError, 0,
      0, estimationError; // 估计误差协方差矩阵
  I.setIdentity();        // 单位矩阵
  x.setZero();            // 状态向量初始化为零
}
void KalmanFilter::predict(float u)
{
  x = A * x + B * u;
  P = A * P * A.transpose() + Q;
}
void KalmanFilter::update(float z)
{
  Eigen::Matrix<float, 1, 1> z_matrix;
  z_matrix(0, 0) = z;
  Eigen::Matrix<float, 1, 1> y = z_matrix - H * x;                // 计算测量预测误差
  Eigen::Matrix<float, 1, 1> S = H * P * H.transpose() + R;       // 计算测量预测误差协方差
  Eigen::Matrix<float, 2, 1> K = P * H.transpose() * S.inverse(); // 计算卡尔曼增益
  x = x + K * y;                                                  // 更新状态向量
  P = (I - K * H) * P;                                            // 更新估计误差协方差矩阵
}
float KalmanFilter::getPosition()
{
  return x(0); // 返回位置
}
float KalmanFilter::getVelocity()
{
  return x(1); // 返回速度
}

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

// oplin add for 巡检
void Custom::rise()
{
  cmdReset();
  cmd.mode = 1;
  cmd.euler[1] = -0.60;
  sleep(3);
  cmdReset(); // TODO 如果接入整体运动控制,这个cmdReset是不是应该去掉?
}
void Custom::lower()
{
  cmdReset();
  cmd.mode = 1;
  cmd.euler[1] = 0.60;
  sleep(3);
  cmdReset();
}
// void Custom::warning() {
//     cmdReset();
//     FaceLightClient client;
//     /* Same Color Test */
//     client.setAllLed(client.red);
//     client.sendCmd();
//     usleep(1200000);
//     client.setAllLed(client.blue);
//     client.sendCmd();
//     usleep(1200000);
//     client.setAllLed(client.red);
//     client.sendCmd();
//     usleep(1200000);
//     client.setAllLed(client.blue);
//     client.sendCmd();
//     usleep(1200000);
//     client.setAllLed(client.black);
//     client.sendCmd();
// }

double degreesToRadians(double degrees)
{
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

void Custom::moveLeft(float distance, float speed)
{
  udp.GetRecv(state);
  float initialPositionY = state.position[1];
  float currentPositionY = initialPositionY;
  float targetPositionY = initialPositionY + distance;
  float accelY = 0.0;
  KalmanFilter kf(0.1, 0.1, 1.0);    // 初始化卡尔曼滤波器
  const float updateInterval = 0.01; // 更新间隔（秒）
  PIDController pidY(0.5, 0, 0);
  cmdReset();
  cmd.mode = 2;
  cmd.gaitType = 1;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
    udp.GetRecv(state);
    currentPositionY = state.position[1];
    accelY = state.imu.accelerometer[1];
    kf.predict(accelY * updateInterval); // 预测步骤
    kf.update(currentPositionY);         // 更新步骤
    float filteredPositionY = kf.getPosition();
    float offset = 0.1;
    if (std::abs(filteredPositionY - initialPositionY) + offset >= distance)
      break;
    float speedY = pidY.P(targetPositionY, filteredPositionY);
    setVelocity(0, speedY, 0);
  }
  cmdReset();
}
void Custom::moveForward(float distance, float speed)
{
    udp.GetRecv(state);
    float initialPositionX = state.position[0];
    float currentPositionX = initialPositionX;
    float targetPositionX = initialPositionX + distance;
    float accelX = 0.0;
    KalmanFilter kf(0.1, 0.1, 1.0); // 初始化卡尔曼滤波器
    const float updateInterval = 0.01; // 更新间隔（秒）
    PIDController pidX(0.5, 0, 0);
    cmdReset();
    cmd.mode = 2;
    cmd.gaitType = 1;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
        udp.GetRecv(state);
        currentPositionX = state.position[0];
        accelX = state.imu.accelerometer[0];
        kf.predict(accelX * updateInterval); // 预测步骤
        kf.update(currentPositionX);         // 更新步骤
        float filteredPositionX = kf.getPosition();
        float offset = 0.1;
        std::cout << "filteredPositionX: " << filteredPositionX << std::endl;
        std::cout << "initialPositionX: " << initialPositionX << std::endl;
        if (std::abs(filteredPositionX - targetPositionX) <= offset)
            break;
        float speedX = pidX.P(targetPositionX, filteredPositionX);
        std::cout << "speedX: " << speedX << std::endl;
        setVelocity(speedX, 0, 0);
    }
    cmdReset();
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
