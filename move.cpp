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
		0, 1;	 // 状态转移矩阵
	B << 0.5, 1; // 控制输入矩阵
	H << 1, 0;	 // 测量矩阵
	Q << processNoise, 0,
		0, processNoise;   // 过程噪声协方差矩阵
	R << measurementNoise; // 测量噪声协方差矩阵
	P << estimationError, 0,
		0, estimationError; // 估计误差协方差矩阵
	I.setIdentity();		// 单位矩阵
	x.setZero();			// 状态向量初始化为零
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
	Eigen::Matrix<float, 1, 1> y = z_matrix - H * x;				// 计算测量预测误差
	Eigen::Matrix<float, 1, 1> S = H * P * H.transpose() + R;		// 计算测量预测误差协方差
	Eigen::Matrix<float, 2, 1> K = P * H.transpose() * S.inverse(); // 计算卡尔曼增益
	x = x + K * y;													// 更新状态向量
	P = (I - K * H) * P;											// 更新估计误差协方差矩阵
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
	if (abs(cmd.velocity[0]) < 1 && abs(cmd.velocity[1]) < 1 && abs(cmd.yawSpeed) < 1)
	{
		udp.SetSend(cmd);
	}
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

void Custom::setVelocity(float vx, float vy, float vr, int gait)
{
	cmdReset();
	cmd.mode = 2;
	cmd.gaitType = gait;
	cmd.velocity[0] = vx;
	cmd.velocity[1] = vy;
	cmd.yawSpeed = vr;
	std::cout << "vx: " << vx << " vy: " << vy << " vr: " << vr << std::endl;
}
void Custom::setVelocityClimb(float vx, float vy, float vr)
{
	cmdReset();
	cmd.mode = 2;
	cmd.gaitType = 3;
	cmd.velocity[0] = vx;
	cmd.velocity[1] = vy;
	cmd.yawSpeed = vr;
	std::cout << "Climb vx: " << vx << " vy: " << vy << " vr: " << vr << std::endl;
}

void Custom::moveLeft(float distance, float speed)
{
	udp.GetRecv(state);
	float initialPositionY = state.position[1];
	std::cout << "initialPositionY: " << initialPositionY << std::endl;
	float currentPositionY = initialPositionY;
	float targetPositionY = initialPositionY + distance;
	float accelY = 0.0;
	KalmanFilter kf(0.1, 0.1, 1.0);	   // 初始化卡尔曼滤波器
	const float updateInterval = 0.01; // 更新间隔（秒）
	PIDController pidY(0.5, 0, 0);
	cmdReset();
	while (true)
	{
		udp.GetRecv(state);
		currentPositionY = state.position[1];
		accelY = state.imu.accelerometer[1];
		kf.predict(accelY * updateInterval); // 预测步骤
		kf.update(currentPositionY);		 // 更新步骤
		float filteredPositionY = kf.getPosition();
		float tolerance = 0.1;
		std::cout << "targetPositionY: " << targetPositionY << std::endl;
		std::cout << "filteredPositionY: " << filteredPositionY << std::endl;
		if (std::abs(filteredPositionY - targetPositionY) <= tolerance)
			break;
		double speedY = pidY.P(targetPositionY, filteredPositionY);
		speedY = min(speedY, 0.3), speedY = max(speedY, -0.3);
		setVelocity(0, speedY, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
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
	KalmanFilter kf(0.1, 0.1, 1.0);	   // 初始化卡尔曼滤波器
	const float updateInterval = 0.01; // 更新间隔（秒）
	PIDController pidX(0.5, 0, 0);
	cmdReset();
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		udp.GetRecv(state);
		currentPositionX = state.position[0];
		accelX = state.imu.accelerometer[0];
		kf.predict(accelX * updateInterval); // 预测步骤
		kf.update(currentPositionX);		 // 更新步骤
		float filteredPositionX = kf.getPosition();
		float offset = 0.1;
		std::cout << "filteredPositionX: " << filteredPositionX << std::endl;
		std::cout << "targetPositionX: " << targetPositionX << std::endl;
		if (std::abs(filteredPositionX - targetPositionX) <= offset)
			break;
		double speedX = pidX.P(targetPositionX, filteredPositionX);
		speedX = min(speedX, 0.3), speedX = max(speedX, -0.3);
		std::cout << "speedX: " << speedX << std::endl;
		setVelocity(speedX, 0, 0);
	}
	cmdReset();
}

void Custom::rotateLeft(float angle, float speed)
{
	// 角度转换为弧度
	float speedRad = degreesToRadians(speed);
	float angleRad = degreesToRadians(angle);
	udp.GetRecv(state);
	float initialYaw = state.imu.rpy[2];
	float targetYaw = initialYaw + angleRad;
	const float updateInterval = 0.01; // 更新间隔（秒）
	cmdReset();
	PIDController pidYaw(1, 0, 0);
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		udp.GetRecv(state);
		float currentYaw = state.imu.rpy[2];
		// Normalize currentYaw and targetYaw to the range [-pi, pi]
		float yawDifference = targetYaw - currentYaw;
		while (yawDifference > M_PI)
			yawDifference -= 2 * M_PI;
		while (yawDifference < -M_PI)
			yawDifference += 2 * M_PI;
		// Normalize currentYaw to the range [-pi, pi]
		while (currentYaw > M_PI)
			currentYaw -= 2 * M_PI;
		while (currentYaw < -M_PI)
			currentYaw += 2 * M_PI;
		float tolerance = 0.1;
		if (std::abs(yawDifference) <= tolerance) // 停止条件
		{
			std::cout << "Initial Yaw: " << initialYaw << std::endl;
			std::cout << "Final Yaw: " << currentYaw << std::endl;
			break;
		}
		double speedYaw = pidYaw.P(targetYaw, currentYaw);
		speedYaw = min(speedYaw, 0.5), speedYaw = max(speedYaw, -0.5);
		if (angle < 0)
			setVelocity(0, 0, speedYaw);
		else
			setVelocity(0.04, 0, speedYaw);
	}
	cmdReset();
}

void Custom::forwardWalkNew(float distance, float speed, int gait)
{
	cmdReset();
	double minus = 1;
	if (distance < 0)
		minus = -1;
	distance = abs(distance);
	udp.GetRecv(state);
	float initialPositionX = state.position[0];
	float initialPositionY = state.position[1];
	float initialYaw = state.imu.rpy[2]; // 获取初始朝向角度
	const float updateInterval = 0.01;	 // 更新间隔（秒）
	KalmanFilter kfX(0.1, 0.1, 1.0);	 // 初始化X方向卡尔曼滤波器
	KalmanFilter kfY(0.1, 0.1, 1.0);	 // 初始化Y方向卡尔曼滤波器
	PIDController pidX(1, 0, 0);
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		udp.GetRecv(state);
		float currentPositionX = state.position[0];
		float currentPositionY = state.position[1];
		float accelX = state.imu.accelerometer[0];
		float accelY = state.imu.accelerometer[1];
		float currentYaw = state.imu.rpy[2];  // 获取当前朝向角度
		kfX.predict(accelX * updateInterval); // 预测步骤X方向
		kfX.update(currentPositionX);		  // 更新步骤X方向
		kfY.predict(accelY * updateInterval); // 预测步骤Y方向
		kfY.update(currentPositionY);		  // 更新步骤Y方向
		float filteredPositionX = kfX.getPosition();
		float filteredPositionY = kfY.getPosition();
		std::cout << "Filtered Position: X=" << filteredPositionX << " Y=" << filteredPositionY << std::endl;
		// 计算当前相对于初始位置和朝向的位移
		float deltaX = filteredPositionX - initialPositionX;
		float deltaY = filteredPositionY - initialPositionY;
		// 将位移转换到初始坐标系中
		float relativeX = deltaX * cos(initialYaw) + deltaY * sin(initialYaw);
		float relativeY = deltaY * cos(initialYaw) - deltaX * sin(initialYaw);
		// 计算与目标位置的距离
		float distanceTraveled = sqrt(pow(relativeX, 2) + pow(relativeY, 2));
		std::cout << "distanceTraveled: " << distanceTraveled << std::endl;
		std::cout << "distance: " << distance << std::endl;
		// 检查是否达到目标距离
		double tolerance = 0.05;
		if (std::abs(distance - distanceTraveled) <= tolerance)
		{
			std::cout << "Final Position: X=" << state.position[0] << " Y=" << state.position[1] << " Z=" << state.position[2] << std::endl;
			break;
		}
		// 保持X方向速度
		// double speedX = pidX.P(distance, distanceTraveled) * minus;
		double speedX = speed * minus;
		speedX = min(speedX, 0.3), speedX = max(speedX, -0.3);
		setVelocity(speedX, 0, 0, gait);
	}
	cmdReset();
}
void Custom::leftWalkNew(float distance, float speed)
{
	cmdReset();
	double minus = 1;
	if (distance < 0)
		minus = -1;
	distance = abs(distance);
	udp.GetRecv(state);
	float initialPositionX = state.position[0];
	float initialPositionY = state.position[1];
	float initialYaw = state.imu.rpy[2]; // 获取初始朝向角度
	const float updateInterval = 0.01;	 // 更新间隔（秒）
	KalmanFilter kfX(0.1, 0.1, 1.0);	 // 初始化X方向卡尔曼滤波器
	KalmanFilter kfY(0.1, 0.1, 1.0);	 // 初始化Y方向卡尔曼滤波器
	PIDController pidY(1, 0, 0);
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		std::cout << "distance: " << distance << std::endl;
		udp.GetRecv(state);
		float currentPositionX = state.position[0];
		float currentPositionY = state.position[1];
		float accelX = state.imu.accelerometer[0];
		float accelY = state.imu.accelerometer[1];
		float currentYaw = state.imu.rpy[2];  // 获取当前朝向角度
		kfX.predict(accelX * updateInterval); // 预测步骤X方向
		kfY.predict(accelY * updateInterval); // 预测步骤Y方向
		kfX.update(currentPositionX);		  // 更新步骤X方向
		kfY.update(currentPositionY);		  // 更新步骤Y方向
		float filteredPositionX = kfX.getPosition();
		float filteredPositionY = kfY.getPosition();
		// std::cout << "Filtered Position: X=" << filteredPositionX << " Y=" << filteredPositionY << std::endl;
		// 计算当前相对于初始位置和朝向的位移
		float deltaX = filteredPositionX - initialPositionX;
		float deltaY = filteredPositionY - initialPositionY;
		// 将位移转换到初始坐标系中
		float relativeX = deltaX * cos(initialYaw) + deltaY * sin(initialYaw);
		float relativeY = deltaY * cos(initialYaw) - deltaX * sin(initialYaw);
		// 计算与目标位置的距离
		float distanceTraveled = sqrt(pow(relativeX, 2) + pow(relativeY, 2));
		std::cout << "distanceTraveled: " << distanceTraveled << std::endl;
		std::cout << "distance: " << distance << std::endl;
		// 检查是否达到目标距离
		double tolerance = 0.05;
		if (std::abs(distance - distanceTraveled) <= tolerance)
		{
			std::cout << "Final Position: X=" << state.position[0] << " Y=" << state.position[1] << " Z=" << state.position[2] << std::endl;
			break;
		}
		// 保持Y方向速度
		double speedY = pidY.P(distance, distanceTraveled) * minus;
		std::cout << "speedY: " << speedY << std::endl;
		speedY = min(speedY, 0.3), speedY = max(speedY, -0.3);
		setVelocity(0, speedY, 0);
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
