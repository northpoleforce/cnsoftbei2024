#include <opencv2/opencv.hpp>

#include <Camera.h>
#include <MapImageProcessing.h>

int main()
{
    Camera cam2(2);
    // 创建一个BGRMaskGenerator对象
    // BGRMaskGenerator maskGenerator(30, 90, 30, 90, 30, 90);
    BGRMaskGenerator maskGenerator(0, 90, 0, 90, 0, 90);
    ImageProcessor imageProcessor;
    while (1)
    {
        // // 读取一张图像
        // cv::Mat img = cv::imread("./dataset/test/00.jpg", cv::IMREAD_COLOR);
        // cv::Mat img = cv::imread("./dataset/test/02.jpg", cv::IMREAD_COLOR);
        cv::Mat img = cam2.getFrame();
        if (img.empty())
        {
            std::cerr << "Error: Image not found" << std::endl;
            return 1;
        }
        // cv::Mat img = cam2.getFrame();
        // // 显示图像
        // cv::imshow("Image", img);
        // 获取mask
        cv::Mat mask = maskGenerator.getMask(img);
        
        // 定义结构元素，这里使用3x3的矩形
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat erodedMask;
        cv::erode(mask, erodedMask, element);
        // cv::Mat result;
        // // 进行开运算
        // cv::morphologyEx(mask, result, cv::MORPH_OPEN, element);
        mask = erodedMask.clone();
        // 二值图转彩色图
        cv::Mat Mask0;
        cv::cvtColor(mask, Mask0, cv::COLOR_GRAY2BGR);
        // cv::imshow("Mask0", mask);
        mask = imageProcessor.createCustomMask(mask.clone());
        cv::Mat Mask1;
        cv::cvtColor(mask, Mask1, cv::COLOR_GRAY2BGR);
        // // 显示mask
        // cv::imshow("Mask1", mask);

        // 进行Canny边缘检测
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 150);
        // 将二值化图像转换为彩色图像
        cv::Mat colorMask;
        cv::cvtColor(edges, colorMask, cv::COLOR_GRAY2BGR);
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 100, 25);
        // 在彩色图像上绘制检测到的直线
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line(colorMask, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
        // 输出直线的数量
        // std::cout << "Number of lines: " << lines.size() << std::endl;

        // 创建一个map来存储角度相同的线
        std::map<int, std::vector<cv::Vec4i>> linesByDegree;
        std::map<int, int> linesByDegreeCount;
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            double dx = l[2] - l[0]; // x2 - x1
            double dy = l[3] - l[1]; // y2 - y1
            // 计算角度（弧度）
            double radian = std::atan2(dy, dx);
            // 将弧度转换为角度
            double degree = radian * (180.0 / CV_PI);
            // 输出每条直线的角度
            // std::cout << "Line " << i << ": " << degree << " degrees" << std::endl;
            // 把角度四舍五入到最近的整数
            int roundedDegree = std::round(degree);
            // std::cout << "Rounded degree: " << roundedDegree << " degrees" << std::endl;
            // 把线添加到对应角度的vector中
            linesByDegree[roundedDegree].push_back(l);
            linesByDegreeCount[roundedDegree]++;
        }
        // 补偿接近竖直的计算误差
        for (int i = 88; i <= 90; ++i)
        {
            if (linesByDegree.find(i) != linesByDegree.end())
            {
                // 获取i的数量
                int amount = linesByDegree[i].size();
                // 获取i的元素
                std::vector<cv::Vec4i> lines = linesByDegree[i];
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
        // // 输出每个角度的数量
        // for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        // {
        //     std::cout << "Degree: " << it->first << " degrees" << std::endl;
        //     std::cout << "Number of lines: " << it->second << std::endl;
        // }
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
        // // 把linesByDegree按数量排序
        // std::vector<std::pair<int, std::vector<cv::Vec4i>>> sortedLinesByDegree(linesByDegree.begin(), linesByDegree.end());
        // std::sort(sortedLinesByDegree.begin(), sortedLinesByDegree.end(), [](const std::pair<int, std::vector<cv::Vec4i>> &a, const std::pair<int, std::vector<cv::Vec4i>> &b)
        //           { return a.second.size() > b.second.size(); });
        // // 输出排序后的角度和数量
        // for (size_t i = 0; i < sortedLinesByDegree.size(); i++)
        // {
        //     std::cout << "Degree: " << sortedLinesByDegree[i].first << " degrees" << std::endl;
        //     std::cout << "Number of lines: " << sortedLinesByDegree[i].second.size() << std::endl;
        // }
        // 找第一个数量多的角度和第二个数量多的角度
        int firstDegree = 0;
        int secondDegree = 0;
        int firstDegreeCount = 0;
        int secondDegreeCount = 0;
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
        std::vector<cv::Vec4i> firstDegreeLines = linesByDegree[firstDegree];
        // // 明确分类，第一多角度小于等于90度为一种处理方式，大于90度为另一种处理方式
        // if (firstDegree <= 90)
        // 把参数传入函数，传入角度和线
        {
            std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(firstDegree, firstDegreeLines);
            cv::Point leftPoint = lineEnds.first;
            cv::Point rightPoint = lineEnds.second;
            // 输出这两个点
            // std::cout << "Left point: " << leftPoint << std::endl;
            // std::cout << "Right point: " << rightPoint << std::endl;
            // 计算两点的中点
            cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
            // 输出中点
            // std::cout << "Mid point: " << midPoint << std::endl;
            // 以firstDegree作为角度，midPoint作为中点，绘制一条直线
            double radian = firstDegree * (CV_PI / 180.0);
            // std::cout << "Radian: " << radian << std::endl;
            double dx = std::cos(radian);
            double dy = std::sin(radian);
            // std::cout << "dx: " << dx << std::endl;
            // std::cout << "dy: " << dy << std::endl;
            cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
            cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
            // std::cout << "p1: " << p1 << std::endl;
            // std::cout << "p2: " << p2 << std::endl;
            cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }
        // 对第二多的角度的线进行拟合
        std::vector<cv::Vec4i> secondDegreeLines = linesByDegree[secondDegree];
        // // 明确分类，第二多角度小于等于90度为一种处理方式，大于90度为另一种处理方式
        // if (secondDegree <= 90)
        {
            // 计算两点的中点
            std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(secondDegree, secondDegreeLines);
            cv::Point leftPoint = lineEnds.first;
            cv::Point rightPoint = lineEnds.second;
            cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
            // std::cout << "Left point: " << leftPoint << std::endl;
            // std::cout << "Right point: " << rightPoint << std::endl;
            // std::cout << "Mid point: " << midPoint << std::endl;
            // 以secondDegree作为角度，midPoint作为中点，绘制一条直线
            double radian = secondDegree * (CV_PI / 180.0);
            double dx = std::cos(radian);
            double dy = std::sin(radian);
            cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
            cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
            // std::cout << "p1: " << p1 << std::endl;
            // std::cout << "p2: " << p2 << std::endl;
            cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }

        // // 显示检测到的直线
        // cv::imshow("Detected Lines", colorMask);

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

        cv::waitKey(1);
    }

    return 0;
}