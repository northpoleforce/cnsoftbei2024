#include "move.h"

int main()
{
    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    while (1) {
        float degree, speed;
        float distance;

        // std::cout << "\033[1;31m turn left: Please input the degree and speed: \033[0m" << std::endl;
        // std::cin >> degree >> speed;
        // custom.turnLeftDegree(degree, speed);
        // std::cout << "\033[1;31m turn right: Please input the degree and speed: \033[0m" << std::endl;
        // std::cin >> degree >> speed;
        // custom.turnRightDegree(degree, speed);

        std::cout << "\033[1;31m forward walk: Please input the distance and speed: \033[0m" << std::endl;
        std::cin >> distance >> speed;
        custom.forwardWalk_m(distance, speed);
        // std::cout << "\033[1;31m left walk: Please input the distance and speed: \033[0m" << std::endl;
        // std::cin >> distance >> speed;
        // custom.leftWalk_m(distance, speed);
        // std::cout << "\033[1;31m right walk: Please input the distance and speed: \033[0m" << std::endl;
        // std::cin >> distance >> speed;
        // custom.rightWalk_m(distance, speed);

        // std::cout << "\033[1;31m left move: Please input : gaitType, distance and speed: \033[0m" << std::endl;
        // int gType;
        // std::cin >> gType >> distance >> speed;
        // custom.leftmove_test(gType, distance, speed);

        // std::cout << "\033[1;31m Turn left 90 \033[0m" << std::endl;
        // std::cin >> speed;
        // custom.turnLeft90();

        // custom.showIMU();
    }
}