#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Camera.h>

#include <unistd.h>

int main()
{
    // char cwd[1024];
    // if (getcwd(cwd, sizeof(cwd)) != NULL)
    // {
    //     std::cout << "Current working directory: " << cwd << std::endl;
    // }
    // else
    // {
    //     std::cerr << "getcwd() error" << std::endl;
    //     return 1;
    // }

    // // 读取源图像和模板图像
    // cv::Mat templ = cv::imread("./dataset/template/1.png", cv::IMREAD_COLOR);

    // 读取模板图像
    std::vector<cv::Mat> templates;
    for (int i = 1; i <= 4; ++i)
    {
        cv::Mat templ = cv::imread("./dataset/template/" + std::to_string(i) + ".png", cv::IMREAD_COLOR);
        if (templ.empty())
        {
            std::cout << "Could not open or find the image" << std::endl;
            return -1;
        }
        templ.convertTo(templ, CV_8U);
        templates.push_back(templ);
    }

    Camera cam(2);
    std::vector<cv::Scalar> colors = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0)};
    std::vector<std::string> descriptions = {"Right Turn", "Left Turn", "Fork Ahead", "End of Fork"};
    while (1)
    {
        double t_start = cv::getTickCount(); // 开始时间
        cv::Mat frame = cam.getFrame();

        if (frame.empty())
        {
            std::cout << "Frame is empty!" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Frame is not empty! Dedecting!" << std::endl;
        }

        cv::Mat img = frame.clone();
        img.convertTo(img, CV_8U);

        for (int i = 0; i < templates.size(); ++i)
        {
            double t_start = cv::getTickCount(); // 开始时间
            const auto &templ = templates[i];
            // 旋转，找到最佳匹配位置
            double bestConfidence = 0;
            cv::Point bestLoc;
            cv::Mat bestRotatedTempl;
            for (int angle = 0; angle < 360; angle += 20) // 旋转角度，每次旋转10度
            {
                // 旋转模板
                cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point(templ.cols / 2, templ.rows / 2), angle, 1);
                cv::Mat rotatedTempl;
                cv::warpAffine(templ, rotatedTempl, rotMat, templ.size());

                // 创建结果矩阵
                cv::Mat result;
                int result_cols = img.cols - rotatedTempl.cols + 1;
                int result_rows = img.rows - rotatedTempl.rows + 1;
                result.create(result_rows, result_cols, CV_32FC1);

                // 进行模板匹配
                cv::matchTemplate(img, rotatedTempl, result, cv::TM_CCOEFF_NORMED);

                // 通过阈值找到最佳匹配位置
                double minVal, maxVal;
                cv::Point minLoc, maxLoc;
                cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

                // 寻找最佳的匹配位置
                if (maxVal > bestConfidence)
                {
                    bestConfidence = maxVal;
                    bestLoc = maxLoc;
                    bestRotatedTempl = rotatedTempl;
                }
                // // 使用矩形标记出匹配区域
                // cv::rectangle(img, maxLoc, cv::Point(maxLoc.x + rotatedTempl.cols, maxLoc.y + rotatedTempl.rows), colors[i], 2, 8, 0);
                // // 标出置信度和描述
                // std::string text = descriptions[i] + ", Confidence: " + std::to_string(maxVal);
                // cv::putText(img, text, cv::Point(img.cols - 300, 30 + 30 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 2);
                // // 显示置信度
                // std::cout << text << std::endl;
            }
            // 使用矩形标记出匹配区域
            cv::rectangle(img, bestLoc, cv::Point(bestLoc.x + bestRotatedTempl.cols, bestLoc.y + bestRotatedTempl.rows), colors[i], 2, 8, 0);
            // 标出置信度和描述
            std::string text = descriptions[i] + ", Confidence: " + std::to_string(bestConfidence);
            cv::putText(img, text, cv::Point(img.cols - 300, 30 + 30 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 2);
        }
        double t_end = cv::getTickCount();                             // 结束时间
        double t = ((double)t_end - t_start) / cv::getTickFrequency(); // 计算处理时间
        double fps = 1.0 / t;                                          // 计算帧率
        // 在图像上显示帧率
        std::string fps_text = "FPS: " + std::to_string(fps);
        cv::putText(img, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        // 显示结果
        cv::imshow("Match Result", img);
        cv::waitKey(1);
    }

    return 0;
}