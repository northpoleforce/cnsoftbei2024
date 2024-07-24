#include <Camera.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

int main()
{
    Camera cam2(2);
    // Camera cam5(5);
    cv::Mat frame2, frame5;
    // 每隔3s保存一次frame2，文件名加上时间戳
    // 用时间
    auto start = std::chrono::system_clock::now();
    while (true)
    {
        int count = 0;
        frame2 = cam2.getFrame();
        // frame5 = cam5.getFrame();

        if (frame2.empty())
        {
            std::cout << "Frame is empty!" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Frame is not empty! Dedecting!" << std::endl;
        }
        cv::imshow("Camera 2", frame2);

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (static_cast<int>(elapsed_seconds.count()) % 3 == 0)
        {
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::stringstream ss;
            // ss << std::ctime(&end_time) << ".jpg";
            ss << "./dataset/" << std::ctime(&end_time) << ".jpg";
            // ss << std::ctime(&end_time) << ".jpg";
            cv::imwrite(ss.str(), frame2);
        }
        start = std::chrono::system_clock::now();
    }

    return 0;
}