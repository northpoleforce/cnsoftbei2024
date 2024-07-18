#include "DogMotion.h"


DogMotion::DogMotion(uint8_t level) : safe(LeggedType::Go1), udp(level, 8090, "192.168.123.161", 8082) {
    udp.InitCmdData(cmd);
    // 在构造类的时候开始接受信号

};


void DogMotion::UDPRecv()
{
    // wait();
    udp.Recv();
}

void DogMotion::UDPSend()
{
    // wait();
    udp.Send();
}

void DogMotion::getInitIMU(int time) {
    initState();
    // mode
    // 0：空闲模式，默认站立
    // 1：强制站立，由 dBodyHeight + ypr 控制
    // 2：目标速度行走，由 velocity + yawSpeed 控制
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    // gaitType
    // 0: 空闲，机器人不进行任何动作。
    // 1: 小跑，机器人以小跑的方式移动。
    // 2: 小跑运行，机器人以更快的小跑方式移动。
    // 3: 爬楼梯，机器人以爬楼梯的方式移动。
    // 4: 超越障碍的小跑，机器人以小跑的方式移动并超越障碍。
    cmd.gaitType = 1;
    // isFirst：false
    // initYaw：0
    while (time--) {
        // 获取的yaw的初始值
        // 如果不是第一次，并且初始yaw值为0
        if (!isFirst && initYaw==0) {
            udp.GetRecv(state);
            initYaw = state.imu.rpy[2] * (180.0 / M_PI); // 获得初始yaw值
            if (initYaw != 0) {
                isFirst = true;
                cout << "初始Yaw值为： " << initYaw << endl;
            }
        }
        //cmd.velocity[0] = 0.1f;
//            yawCorrect(90);
        // 每10000微秒设置一次发送命令（即10毫秒，0.01秒）
        usleep(10000);
        udp.SetSend(cmd); // really necessary?
    }
}

// 状态初始化
void DogMotion::initState()
{
    // 控制模式
    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    // 步态选择
    cmd.gaitType = 0;
    // 速度等级（目前没有开发）
    cmd.speedLevel = 0;
    // 抬脚高度（-0.06m ~ 0.09m）
    cmd.footRaiseHeight = 0;
    // 身体高度（-0.13m ~ 0.03m）
    cmd.bodyHeight = 0;
    // 在站立模式下的欧拉角（弧度制）
    // 0： 横滚角：（-0.75~0.75)
    // 1： 俯仰角：（-0.75~0.75)
    // (-42 ~ 42 degree)
    // 2： 偏航角：（-0.6~0.6)
    // (-34 ~ 34 degree)
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    // 在机体坐标系下的速度：0：前后速度，1：左右速度
    // (range: 行走 : vx:-1.1~1.5m/s,  vy:-1.0~1.0m/s)
    // (range: 跑步  : vx:-2.5~3.5m/s,  vy:-1.0~1.0m/s)
    // (range: 爬楼梯: vx:-0.2~0.25m/s, vy:-0.15~0.15m/s)
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = -0.0125f;//DO NOT ADJUST DEFAULT VALUE
    // 在机体坐标系下的旋转角速度
    // (range: 行走 : -4.0~4.0rad/s)
    // (range: 跑步  : -4.0~4.0rad/s)
    // (range: 爬楼梯: -0.7~0.7rad/s)
    cmd.yawSpeed = 0.012f;
    cmd.reserve = 0;
}

// 视觉控制
void DogMotion::visionControl() {
    // 运动控制参数初始化
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.012f;
    // below are last competition's vision code, order to let the dog in charge of the vision
    if(isVision && midShiftRight && !yellowRight && !yellowLeft && !greenLeft && !greenRight) {
        // 右移
        cmd.velocity[1] = -0.08f;
        // 前进
        cmd.velocity[0] = 0.05f;
    }
    if(isVision && midShiftLeft && !yellowRight && !yellowLeft && !greenLeft && !greenRight) {
        // 左移
        cmd.velocity[1] = 0.08f;
        // 前进
        cmd.velocity[0] = 0.05f;
    }
    if(isVision && yellowRight && !greenLeft && !greenRight) {
        
        cmd.velocity[0] = 0.05f;
        // 右旋
        cmd.yawSpeed = -0.1f;
        // 右移
        cmd.velocity[1] = -0.2f;
    }
    if(isVision && yellowLeft && !greenLeft && !greenRight) {
        cmd.velocity[0] = 0.05f;
        // 左旋
        cmd.yawSpeed = 0.1f;
        // 左移
        cmd.velocity[1] = +0.2f;
    }
    if (isVision && greenRight) {
        cmd.velocity[1] = -0.4f;
    }
    if (isVision && greenLeft) {
        cmd.velocity[1] = 0.4f;
    }

};


// 自定义yaw的补偿值来调整yawSpeed
void DogMotion::yawCorrect(int deflection=0) {
    udp.GetRecv(state);
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    int baseYaw = this->initYaw + deflection; // baseYaw 就是初始yaw + 偏移 yaw
    // 把baseYaw范围控制在-180~180之间
    if (baseYaw > 180) {
        baseYaw -=360;
    } else if (baseYaw < -180) {
        baseYaw += 360;
    }
    // 当前yaw
    int nowYaw = state.imu.rpy[2] * (180.0 / M_PI);
    // 计算当前yaw与baseYaw的差值
    int diff = nowYaw - baseYaw;
    if (diff > 180) {
        diff -= 360;
    } else if(diff < -180) {
        diff += 360;
    }
    if (abs(diff) > 3 && nowYaw !=0 && baseYaw != 0) {
        // yaw left + , right -
        cout << "nowYaw: " << nowYaw << "  now baseYaw: " << baseYaw << endl;

        // zheng ni, fu shun
        cout << "现在相对baseYaw的偏角为: " << diff << " 正在进行修正 ！" << endl;
        // 根据差值来调整yawSpeed
        if (diff > 0) {
            cmd.yawSpeed = -0.6f;
        } else {
            cmd.yawSpeed = 0.6f;
        }
    }
    else cmd.yawSpeed = 0.012f;
}

// 前进(0.5m/s)：参数：时间，是否使用IMU修正(默认否)，修正量(默认0)，是否使用视觉(默认是)
void DogMotion::GoForward(int time, bool useIMU=false, int deflection=0, bool useVision=true)
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;

    while(time--)
    {
        // 视觉介入
        if (isVision&&useVision) {
            visionControl();
            ++time;
        } else {
        //无视觉
            printf("Go Forward %d times\n",time);
            // 前进速度0.5m/s
            // 左右速度-0.0125m/s
            cmd.velocity[0] = 0.5f;
            cmd.velocity[1] = -0.0125f;//DO NOT ADJUST DEFAULT VALUE
            // 旋转速度0.012rad/s即0.69度/s
            cmd.yawSpeed = 0.012f;
            if (useIMU) yawCorrect(deflection);
        }
        udp.SetSend(cmd);
        usleep(10000);
    }
}

// 右平移(0.4m/s)：参数：时间，是否使用视觉(默认是)
void DogMotion::RightTranslation(int time,bool useVision=true)
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    while(time--)
    {
        if (isVision&&useVision) {
            visionControl();
            ++time;
        } else {
            printf("Right Translation %d times\n",time);
            cmd.velocity[1] = -0.4f;
        }
        udp.SetSend(cmd);
        usleep(10000);
    }
}
// 左平移(0.4m/s)：参数：时间，是否使用视觉(默认是)
void DogMotion::LeftTranslation(int time,bool useVision=true)
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;//if dog go foward continuously when translation, change mode to another digit

    while(time--)
    {
        if (isVision&&useVision) {
            visionControl();
            ++time;
        } else {
            printf("Left Translation %d times\n",time);
            cmd.velocity[1] = 0.4f;
        }
        udp.SetSend(cmd);
        usleep(10000);
    }
}

// 右转(直行速度:v0 m/s，旋转速度:66度/s)：参数：时间，速度v0，是否使用视觉(默认是)
void DogMotion::TurnRight(int time,double v0, bool useVision=true)//time_yaw90=175 v0_yaw90=0.45 time_yaw45=110 v0_yaw45=0.4
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    while(time--)
    {
        if (isVision&&useVision) {
            visionControl();
            ++time;
        } else {
            printf("Turn Right %d times\n",time);
            cmd.velocity[0] = 1.0f*v0;
            cmd.yawSpeed = -1.15f;
        }

        udp.SetSend(cmd);
        usleep(10000);
    }
}
// 左转(直行速度:v0 m/s，旋转速度:66度/s)：参数：时间，速度v0，是否使用视觉(默认是)
void DogMotion::TurnLeft(int time,double v0, bool useVision=true)//time_yaw90=175 v0_yaw90=0.45 time_yaw45=110 v0_yaw45=0.4
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    while(time--)
    {
        if (isVision&&useVision) {
            visionControl();
            ++time;
        } else {
            cout << "now yaw:" << state.imu.rpy[2] * (180.0 / 3.1415926) << endl;
            printf("Turn Left %d times\n",time);
            cmd.velocity[0] = 1.0f*v0;
            cmd.yawSpeed = 1.15f;
        }

        udp.SetSend(cmd);
        usleep(10000);
    }
}

// 右圆转(直行速度:v0 m/s，旋转速度:yaws弧度/s)：参数：时间，速度v0，旋转速度yaws
void DogMotion::RightCircle(int time,double v0,double yaws)
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    while(time--)
    {
        if (!isVision) {
            printf("Right Circle %d times\n",time);
            cmd.velocity[0] = 1.0f*v0;
            cmd.yawSpeed = -1.0f*yaws;
        } else {
            visionControl();
            ++time;
        }

        udp.SetSend(cmd);
        usleep(10000);
    }
}
// 左圆转(直行速度:v0 m/s，旋转速度:yaws弧度/s)：参数：时间，速度v0，旋转速度yaws
void DogMotion::LeftCircle(int time,double v0,double yaws)
{
    initState();
    cmd.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 1;
    while(time--)
    {
        if (!isVision) {
            printf("Left Circle %d times\n",time);
            cmd.velocity[0] = 1.0f*v0;
            cmd.yawSpeed = 1.0f*yaws;
        } else {
            visionControl();
            ++time;
        }

        udp.SetSend(cmd);
        usleep(10000);
    }
}

// 卸载物资：抬头左倾
void DogMotion::Lean_Forward()
{
    // 强制站立
    cmd.mode=1;
    // 前进速度，旋转速度置零
    cmd.velocity[0] = 0;
    cmd.yawSpeed = 0;
    // 俯仰角0.7弧度
    cmd.euler[1] = 0.7f;
    // 发送控制命令并等待1s
    udp.SetSend(cmd);
    sleep(1);
    // 横滚角-1弧度
    cmd.euler[0] = -0.99f;
    // 发送控制命令并等待1s
    udp.SetSend(cmd);
    sleep(1);
    // 俯仰角横滚角置零
    cmd.euler[1] = 0;
    cmd.euler[0] = 0;
    udp.SetSend(cmd);
    sleep(1);
}
// 装载物资：低头右倾
void DogMotion::Lean_Backward()
{
    cmd.mode=1;
    cmd.velocity[0] = 0;
    cmd.yawSpeed = 0;
    cmd.euler[1] = -0.9f;
    udp.SetSend(cmd);
    sleep(1);
    cmd.euler[0] = 0.99f;
    udp.SetSend(cmd);
    sleep(1);
    cmd.euler[1] = 0;
    cmd.euler[0] = 0;
    udp.SetSend(cmd);
    sleep(1);
}

// 任务结束，停止运动
void DogMotion::Stop()
{
    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;//DO NOT ADJUST DEFAULT VALUE
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
    printf("Stop\n");
    udp.SetSend(cmd);
    sleep(100);
}

void DogMotion::setPlanTime(int seconds) {
    this->planTimes = seconds;
}

void DogMotion::RobotControl()
{
    // motiontime 0 未开始
    // motiontime 1 进行中
    if(motiontime<1)
    {
        // below are the sports you want to arrange dog to do

        // 任务状态置为进行中
        motiontime+=1;
        // 1 = 0.01s
        getInitIMU(1);

        //from start to the end of area 1
        // 100 = 1s
        // 左移0.4m(100/100*0.4m/s)，不使用视觉修正
        LeftTranslation(100, false);
        // // 直行0.55m(110/100*0.5m/s)，使用IMU修正，修正量0，不使用视觉修正
        // GoForward(110, true, 0, false);
        GoForward(110, false, 0, false);
        // 右转115.5度，路程0.78m(直行速度0.45*175/100m/s)，不使用视觉修正
        TurnRight(175,0.45, false);
        // 原代码
        // GoForward(110, true, -90, true);
        // 直行0.55m(110/100*0.5m/s)，不使用IMU修正，使用视觉修正
        // GoForward(110, false, 0, true);
        GoForward(110, false, 0, false);
        // 左移0.24m(60/100*0.4m/s)，使用视觉修正(默认)
        // LeftTranslation(60);
        LeftTranslation(60, false);

        //area 1
        // 左圆转：左转2.8弧度，路程1m
        LeftCircle(200,0.5,1.4);
        Lean_Forward();
        // 左圆转：左转4.875弧度，路程1.625m
        LeftCircle(325,0.5,1.5);
        // 右平移：0.28m
        // RightTranslation(70);
        RightTranslation(70, false);

        //from end of area 1 to start of side road
        // 直行1.5m
        GoForward(300);
        // 左转115.5度，路程0.78m
        TurnLeft(175,0.45);
        // 直行1.25m
        GoForward(250);
        // 左转115.5度，路程0.78m
        TurnLeft(175,0.45);
        GoForward(130);

        //side road
        // 岔路
        // 左转72度，路程0.44m
        TurnLeft(110,0.4);//yawleft45-in
        // 右转72度，路程0.44m
        TurnRight(110,0.4);//yawright45-in
        // 直行1.425m
        GoForward(285);
        // 右转69度，路程0.42m
        TurnRight(105,0.4);//yawright45-out
        // 左转72度，路程0.44m
        TurnLeft(110,0.4);//yawleft45-out


        //from end of side road to start of area 2
        // 直行0.725m
        GoForward(145);
        // 左转115.5度，路程0.78m
        TurnLeft(175,0.45);
        // 直行1.75m
        GoForward(350);
        // 左平移0.14m
        LeftTranslation(35);

        //area 2
        // 左圆转：左转148度，路程1m
        LeftCircle(200,0.5,1.3);
        Lean_Forward();
        // 左圆转：左转296度，路程1.85m
        LeftCircle(375,0.5,1.4);

        //from end of area to start of area 3
        // 右移0.2m
        RightTranslation(50);
        // 直行1m
        GoForward(200);
        // 左转115.5度，路程0.78m
        TurnLeft(175,0.45);
        // 右转115.5度，路程0.78m
        TurnRight(175,0.45);
        // 直行1.25m
        GoForward(250);


        //area 3
        Lean_Backward();
        // 右圆转：右转320度，路程1.75m
        RightCircle(350,0.5,1.6);

        //from end of area 3 to start of area 4
        // 直行1.15m
        GoForward(230);
        // 右转59.4度，路程0.36m
        TurnRight(90,0.4);
        // 直行0.6m
        GoForward(120);
        // 左转59.4度，路程0.54m
        TurnLeft(90,0.6);
        // 直行0.35m
        GoForward(75);

        //area 4
        // 左转115.5度，路程0.78m
        TurnLeft(175,0.45);
        // 直行1.4m
        GoForward(280);
        Lean_Backward();
        // 右圆转：右转335度，路程2.25m
        RightCircle(450,0.5,1.3);

        //to the end
        // 直行1.55m
        GoForward(310);
        // 右转115.5度，路程0.78m
        TurnRight(175,0.45);
        // 直行0.575m
        GoForward(115);
        // 右移0.68m
        RightTranslation(170,false);
    }
    Stop();
}
void DogMotion::Run()
{
    // InitEnvironment();
    LoopFunc loop_control("control_loop", dt,    boost::bind(&DogMotion::RobotControl, this));
    LoopFunc loop_udpSend("udp_send",     dt, 3, boost::bind(&DogMotion::UDPSend,      this));
    LoopFunc loop_udpRecv("udp_recv",     dt, 3, boost::bind(&DogMotion::UDPRecv,      this));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    while(1){
        sleep(1);
    };
}
