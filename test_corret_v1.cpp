#include <opencv2/opencv.hpp>

#include <Camera.h>
#include <MapImageProcessing.h>

#include <move.h>

void preTreat(cv::Mat img,
              BGRMaskGenerator maskBGRGenerator, ImageProcessor imageProcessor,
              cv::Mat &mask0Color, cv::Mat &mask1Color, cv::Mat &edges)
{
    // 基于颜色阈值生成掩膜
    cv::Mat mask = maskBGRGenerator.getMask(img);
    // 腐蚀操作
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 定义结构元素，这里使用3x3的矩形
    cv::Mat erodedMask;                                                          // 存储腐蚀后的mask
    cv::erode(mask, erodedMask, element);
    mask = erodedMask.clone();
    cv::cvtColor(mask, mask0Color, cv::COLOR_GRAY2BGR); // 颜色阈值掩膜二值图转彩色图
    // 自定义掩膜
    mask = imageProcessor.createCustomMask(mask);
    cv::cvtColor(mask, mask1Color, cv::COLOR_GRAY2BGR);
    // 进行Canny边缘检测
    cv::Canny(mask, edges, 50, 150);
}

void lineCount(const std::vector<cv::Vec4i> &lines,
               std::map<int, std::vector<cv::Vec4i>> &linesByDegree, std::map<int, int> &linesByDegreeCount)
{
    // 依角度统计线段
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
    // 补偿接近竖直线的统计量
    for (int i = 85; i <= 90; ++i)
    {
        if (linesByDegree.find(i) != linesByDegree.end())
        {

            int amount = linesByDegree[i].size();                   // 获取i的数量
            std::vector<cv::Vec4i> linesPostive = linesByDegree[i]; // 获取i的元素
            std::vector<cv::Vec4i> linesMinus;
            if (linesByDegree.find(-i) != linesByDegree.end())
            {
                // 获取-i的数量
                amount += linesByDegree[-i].size();
                linesMinus = linesByDegree[-i];
                // 更新
                linesByDegreeCount[-i] = amount;
                linesByDegree[-i].insert(linesByDegree[-i].end(), linesPostive.begin(), linesPostive.end());
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
}

int main()
{
    BGRMaskGenerator maskBGRGenerator(0, 90, 0, 90, 0, 90);
    ImageProcessor imageProcessor;
    Camera cam2(2);

    // 启动：运动指令下达
    Custom custom(HIGHLEVEL);
    std::thread mainControl(&Custom::Start, &custom);
    PIDController pidXY(0.0005, 0, 0);
    PIDController pidYaw(0.8, 0, 0);

    while (1)
    {
        cv::Mat img = cam2.getFrame();
        // 图像预处理
        cv::Mat mask0Color, mask1Color, edges;
        preTreat(img, maskBGRGenerator, imageProcessor, mask0Color, mask1Color, edges);
        // 使用霍夫变换检测直线
        const int linelength = 100;
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, linelength, 25);
        // 在彩色图像上绘制检测到的直线
        cv::Mat colorMask;
        cv::cvtColor(edges.clone(), colorMask, cv::COLOR_GRAY2BGR); // 颜色阈值掩膜二值图转彩色图
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line(colorMask, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        // std::cout << "line detected !\n";
        // 用map来存储角度相同的线
        std::map<int, std::vector<cv::Vec4i>> linesByDegree;
        std::map<int, int> linesByDegreeCount;
        lineCount(lines, linesByDegree, linesByDegreeCount);

        // 找数量最多的角度
        int countMax = 0;
        int degreeMaxCount = 0;
        for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        {
            if (it->second > countMax)
            {
                degreeMaxCount = it->first;
                countMax = it->second;
            }
        }
        std::vector<cv::Vec4i> firstDegreeLines = linesByDegree[degreeMaxCount];
        std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(degreeMaxCount, firstDegreeLines);
        cv::Point leftPoint = lineEnds.first;
        cv::Point rightPoint = lineEnds.second;
        cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
        // 以degreeMaxCount作为角度，midPoint作为中点，绘制一条直线
        double radian = degreeMaxCount * (CV_PI / 180.0);
        double dx = std::cos(radian);
        double dy = std::sin(radian);
        cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
        cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
        cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        if (radian < 0) radian += CV_PI;
        custom.setVelocity(0, -pidXY.P(img.cols / 2, midPoint.x), pidYaw.P(CV_PI / 2, radian));
        // std::cout << "draw line\n";
        // midPoint1 = midPoint;
        // k1 = dy / dx;

        // // 找第一个数量多的角度和第二个数量多的角度
        // int firstDegree = 0, secondDegree = 0;
        // int firstDegreeCount = 0, secondDegreeCount = 0;
        // for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        // {
        //     if (it->second > firstDegreeCount)
        //     {
        //         firstDegree = it->first;
        //         firstDegreeCount = it->second;
        //     }
        // }
        // for (auto it = linesByDegreeCount.begin(); it != linesByDegreeCount.end(); it++)
        // {
        //     if (it->second > secondDegreeCount && std::abs(it->first - firstDegree) > 10)
        //     {
        //         secondDegree = it->first;
        //         secondDegreeCount = it->second;
        //     }
        // }
        // // 记录中点和斜率
        // cv::Point midPoint1, midPoint2;
        // double k1, k2;
        // // 第一多的角度的中线
        // {
        //     std::vector<cv::Vec4i> firstDegreeLines = linesByDegree[firstDegree];
        //     std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(firstDegree, firstDegreeLines);
        //     cv::Point leftPoint = lineEnds.first;
        //     cv::Point rightPoint = lineEnds.second;
        //     cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
        //     // 以firstDegree作为角度，midPoint作为中点，绘制一条直线
        //     double radian = firstDegree * (CV_PI / 180.0);
        //     double dx = std::cos(radian);
        //     double dy = std::sin(radian);
        //     cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
        //     cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
        //     cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        //     midPoint1 = midPoint;
        //     k1 = dy / dx;
        // }
        // // 第二多的角度的中线
        // {
        //     std::vector<cv::Vec4i> secondDegreeLines = linesByDegree[secondDegree];
        //     // 计算两点的中点
        //     std::pair<cv::Point, cv::Point> lineEnds = imageProcessor.findLineEndsByDegree(secondDegree, secondDegreeLines);
        //     cv::Point leftPoint = lineEnds.first;
        //     cv::Point rightPoint = lineEnds.second;
        //     cv::Point midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
        //     double radian = secondDegree * (CV_PI / 180.0);
        //     double dx = std::cos(radian);
        //     double dy = std::sin(radian);
        //     cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
        //     cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
        //     cv::line(colorMask, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        //     midPoint2 = midPoint;
        //     k2 = dy / dx;
        // }
        // // 计算两条线的交点
        // double x = (midPoint2.y - midPoint1.y + k1 * midPoint1.x - k2 * midPoint2.x) / (k1 - k2);
        // double y = k1 * (x - midPoint1.x) + midPoint1.y;
        // cv::Point crossPoint(x, y);
        // std::cout << "Cross point: " << crossPoint << std::endl;
        // // 在图像上绘制交点
        // cv::circle(colorMask, crossPoint, 5, cv::Scalar(255, 0, 0), -1);

        // 显示图像
        {
            // 水平拼接 img 和 mask0Color
            cv::Mat horizontal1;
            cv::hconcat(img, mask0Color, horizontal1);
            // std::cout << "horizontal 1\n";
            // 水平拼接 mask1Color 和 colorMask
            cv::Mat horizontal2;
            cv::hconcat(mask1Color, colorMask, horizontal2);
            // 垂直拼接两个水平拼接得到的图像
            // std::cout << "horizontal 2\n";
            cv::Mat finalImage;
            cv::vconcat(horizontal1, horizontal2, finalImage);
            // std::cout << "vertical\n";
            // 显示最终的图像
            cv::imshow("Final Image", finalImage);
        }

        cv::waitKey(1);
    }

    return 0;
}