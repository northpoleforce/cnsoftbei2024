#ifndef MOV_H
#define MOV_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <Eigen/Dense>
// #include "FaceLightClient.h"
using namespace UNITREE_LEGGED_SDK;

class PIDController
{
	double Kp, Ki, Kd;
	double integral = 0.0;
	double previous_error = 0.0;

public:
	PIDController(double Kp, double Ki, double Kd);
	double P(double setpoint, double measured_value);
};
// 卡尔曼滤波器类
class KalmanFilter
{
public:
	KalmanFilter(float processNoise, float measurementNoise, float estimationError);
	void predict(float u);
	void update(float z);
	float getPosition();
	float getVelocity();

private:
	Eigen::Matrix<float, 2, 2> A; // 状态转移矩阵
	Eigen::Matrix<float, 2, 1> B; // 控制输入矩阵
	Eigen::Matrix<float, 1, 2> H; // 测量矩阵
	Eigen::Matrix<float, 2, 2> Q; // 过程噪声协方差矩阵
	Eigen::Matrix<float, 1, 1> R; // 测量噪声协方差矩阵
	Eigen::Matrix<float, 2, 2> P; // 估计误差协方差矩阵
	Eigen::Matrix<float, 2, 2> I; // 单位矩阵
	Eigen::Matrix<float, 2, 1> x; // 状态向量
};

class Custom
{
public:
	Custom(uint8_t level) : safe(LeggedType::Go1),
							udp(level, 8090, "192.168.123.161", 8082)
	{
		udp.InitCmdData(cmd);
	}

	void UDPRecv();
	void UDPSend();
	void RobotControl();

	void cmdReset();

	// oplin add for 巡检任务
	void rise();	// 抬头
	void lower();	// 低头
	void warning(); // 警告

	void turnLeft_Degree(float degree, float speed);
	void turnLeft90();
	void turnRight_Degree(float degree, float speed);

	void forwardWalk_m(float distance, float speed);
	void leftWalk_m(float distance, float speed);
	void rightWalk_m(float distance, float speed);
	void leftmove_test(int gType, float distance, float speed);

	void setVelocity(float vx, float vy, float vr, int gait = 1);
	void setVelocityClimb(float vx, float vy, float vr);
	void moveLeft(float distance, float speed);
	void moveForward(float distance, float speed);
	void rotateLeft(float angle, float speed);
	void forwardWalkNew(float distance, float speed, int gait = 1);
	void leftWalkNew(float distance, float speed);

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