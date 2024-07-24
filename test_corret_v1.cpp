#include <opencv2/opencv.hpp>

#include <Camera.h>
#include <MapImageProcessing.h>

#include <move.h>

int main()
{
    BGRMaskGenerator maskGenerator(0, 90, 0, 90, 0, 90);
    ImageProcessor imageProcessor;
    Camera cam2(2);

    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    PIDController pidXY(0.0005, 0, 0);
    PIDController pidYaw(0.01, 0, 0);

    while (1)
    {
        cv::Mat img = cam2.getFrame();
        cv::imshow("Frame", img);
        cv::waitKey(1);
        // 按下ENTER后才执行下面的代码
        if (cv::waitKey(100) == 13)
        {
            break;
        }
    }

    while (1)
    {
        cv::Mat img = cam2.getFrame();
        // 基于颜色阈值生成掩膜
        cv::Mat mask = maskGenerator.getMask(img);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 定义结构元素，这里使用3x3的矩形
        cv::Mat erodedMask;                                                          // 存储腐蚀后的mask
        cv::erode(mask, erodedMask, element);
        mask = erodedMask.clone();
        cv::Mat Mask0, Mask1;
        cv::cvtColor(mask, Mask0, cv::COLOR_GRAY2BGR); // 颜色二值掩膜转彩色图
        // 自定义掩膜
        mask = imageProcessor.createCustomMask(mask.clone());
        cv::cvtColor(mask, Mask1, cv::COLOR_GRAY2BGR);
        // 进行Canny边缘检测
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 150);
        // 将二值化图像转换为彩色图像
        cv::Mat colorMask;
        cv::cvtColor(edges, colorMask, cv::COLOR_GRAY2BGR);
        // 使用霍夫变换检测直线
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 100, 25);
        // 在彩色图像上绘制检测到的直线
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line(colorMask, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        // 创建一个map来存储角度相同的线
        std::map<int, std::vector<cv::Vec4i>> linesByDegree;
        std::map<int, int> linesByDegreeCount;
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            double dx = l[2] - l[0];                  // x2 - x1
            double dy = l[3] - l[1];                  // y2 - y1
            double radian = std::atan2(dy, dx);       // 计算角度（弧度）
            double degree = radian * (180.0 / CV_PI); // 将弧度转换为角度
            int roundedDegree = std::round(degree);   // 把角度四舍五入到最近的整数
            // 把线添加到对应角度的vector中
            linesByDegree[roundedDegree].push_back(l);
            linesByDegreeCount[roundedDegree]++;
        }
        // 补偿接近竖直的计算误差
        for (int i = 85; i <= 90; ++i)
        {
            if (linesByDegree.find(i) != linesByDegree.end())
            {

                int amount = linesByDegree[i].size();            // 获取i的数量
                std::vector<cv::Vec4i> lines = linesByDegree[i]; // 获取i的元素
                std::vector<cv::Vec4i> linesMinus;
                if (linesByDegree.find(-1) != linesByDegree.end())
                {
                    // 获取-i的数量
                    amount += linesByDegree[-i].size();
                    linesMinus = linesByDegree[-i];
                    // 更新
                    linesByDegreeCount[-i] = amount;
                    linesByDegree[-i].insert(linesByDegree[-i].end(), lines.begin(), lines.end());
                }
                linesByDegreeCount[i] = amount;
                linesByDegree[i].insert(linesByDegree[i].end(), linesMinus.begin(), linesMinus.end());
            }
        }
        // linesByDegree每个角度加上后面3个角度的数量
        auto it_line = linesByDegree.begin();
        for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++, it_line++)
        {
            for (int i = 1; i <= 3; i++)
            {
                int degree = it->first + i;
                if (linesByDegreeCount.find(degree) != linesByDegreeCount.end())
                {
                    it->second += linesByDegreeCount[degree];
                    it_line->second.insert(it_line->second.end(), linesByDegree[degree].begin(), linesByDegree[degree].end());
                }
            }
        }
        // 找第一个数量多的角度和第二个数量多的角度
        int firstDegree = 0, secondDegree = 0;
        int firstDegreeCount = 0, secondDegreeCount = 0;
        for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        {
            if (it->second > firstDegreeCount)
            {
                firstDegree = it->first;
                firstDegreeCount = it->second;
            }
        }
        for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        {
            if (it->second > secondDegreeCount && std::abs(it->first - firstDegree) > 10)
            {
                secondDegree = it->first;
                secondDegreeCount = it->second;
            }
        }
        // 输出第一多和第二多的角度和数量
        // std::cout << "First degree: " << firstDegree << " degrees" << std::endl;
        // std::cout << "First degree count: " << firstDegreeCount << std::endl;
        // std::cout << "Second degree: " << secondDegree << " degrees" << std::endl;
        // std::cout << "Second degree count: " << secondDegreeCount << std::endl;
        // 对第一多的角度的线进行拟合

    // 记录中点和斜率
        cv::Point midPoint1, midPoint2;
        double k1, k2;
        // 第一多的角度的中线
        {
            std::vector<cv::Vec4i> firstDegreeLines = linesByDegree[firstDegree];
            std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(firstDegree, firstDegreeLines);
            cv::Point leftPoint = lineEnds.first;
            cv::Point rightPoint = lineEnds.second;
            cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
            // 以firstDegree作为角度，midPoint作为中点，绘制一条直线
            double radian = firstDegree * (CV_PI / 180.0);
            double dx = std::cos(radian);
            double dy = std::sin(radian);
            cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
            cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
            cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            midPoint1 = midPoint;
            k1 = dy / dx;
        }
        // 第二多的角度的中线
        {
            std::vector<cv::Vec4i> secondDegreeLines = linesByDegree[secondDegree];
            // 计算两点的中点
            std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(secondDegree, secondDegreeLines);
            cv::Point leftPoint = lineEnds.first;
            cv::Point rightPoint = lineEnds.second;
            cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
            double radian = secondDegree * (CV_PI / 180.0);
            double dx = std::cos(radian);
            double dy = std::sin(radian);
            cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
            cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
            cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            midPoint2 = midPoint;
            k2 = dy / dx;
        }
        // 计算两条线的交点
        double x = (midPoint2.y - midPoint1.y + k1 * midPoint1.x - k2 * midPoint2.x) / (k1 - k2);
        double y = k1 * (x - midPoint1.x) + midPoint1.y;
        cv::Point crossPoint(x, y);
        std::cout << "Cross point: " << crossPoint << std::endl;
        if (firstDegree < 0) firstDegree += 180;
        if (secondDegree < 0) secondDegree += 180;
        int degree = std::max(firstDegree, secondDegree);
        std::cout << pidXY.P(img.rows/2, crossPoint.y) << std::endl;
        std::cout << pidXY.P(img.cols/2, crossPoint.x) << std::endl;
        std::cout << "Degree: " << degree << " degrees" << std::endl;
        std::cout << "Yaw_speed: " << pidYaw.P(90, degree) << std::endl;
        // custom.setVelocity(-pidXY.P(img.rows/2, crossPoint.y), -pidXY.P(img.cols/2, crossPoint.x), 0);
        // custom.setVelocity(0, 0, pidYaw.P(90, degree));
        int offset = -img.cols / 8;
        custom.setVelocity(-pidXY.P(img.rows/2, crossPoint.y), -pidXY.P(img.cols/2+offset, crossPoint.x), pidYaw.P(90, degree));
        // 在图像上绘制交点
        cv::circle(colorMask, crossPoint, 5, cv::Scalar(255, 0, 0), -1);
        // 显示图像
        {
            // 水平拼接 img 和 Mask0
            cv::Mat horizontal1;
            cv::hconcat(img, Mask0, horizontal1);
            // 水平拼接 Mask1 和 colorMask
            cv::Mat horizontal2;
            cv::hconcat(Mask1, colorMask, horizontal2);
            // 垂直拼接两个水平拼接得到的图像
            cv::Mat finalImage;
            cv::vconcat(horizontal1, horizontal2, finalImage);
            // 显示最终的图像
            cv::imshow("Final Image", finalImage);
        }

        cv::waitKey(1);
    }

    return 0;
}