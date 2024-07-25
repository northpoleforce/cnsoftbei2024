#include <MapImageProcessing.h>
#include <utility>

HSVMaskGenerator::HSVMaskGenerator(int colorHLower, int colorHUpper,
                                   int colorSLower, int colorSUpper,
                                   int colorVLower, int colorVUpper)
{
    this->colorHLower = colorHLower;
    this->colorHUpper = colorHUpper;
    this->colorSLower = colorSLower;
    this->colorSUpper = colorSUpper;
    this->colorVLower = colorVLower;
    this->colorVUpper = colorVUpper;
}
BGRMaskGenerator::BGRMaskGenerator(int colorBLower, int colorBUpper,
                                   int colorGLower, int colorGUpper,
                                   int colorRLower, int colorRUpper)
{
    this->colorBLower = colorBLower;
    this->colorBUpper = colorBUpper;
    this->colorGLower = colorGLower;
    this->colorGUpper = colorGUpper;
    this->colorRLower = colorRLower;
    this->colorRUpper = colorRUpper;
}
GRAYMaskGenerator::GRAYMaskGenerator(int grayLower, int grayUpper)
{
    this->grayLower = grayLower;
    this->grayUpper = grayUpper;
}
cv::Mat HSVMaskGenerator::getMask(cv::Mat src)
{
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(colorHLower, colorSLower, colorVLower), cv::Scalar(colorHUpper, colorSUpper, colorVUpper), mask);
    return mask;
}
cv::Mat BGRMaskGenerator::getMask(cv::Mat src)
{
    cv::Mat mask;
    cv::inRange(src, cv::Scalar(colorBLower, colorGLower, colorRLower), cv::Scalar(colorBUpper, colorGUpper, colorRUpper), mask);
    return mask;
}
cv::Mat GRAYMaskGenerator::getMask(cv::Mat src)
{
    cv::Mat mask;
    cv::inRange(src, cv::Scalar(grayLower), cv::Scalar(grayUpper), mask);
    return mask;
}

ImageProcessor::ImageProcessor()
{
    const int cols = 640;
    const int rows = 480;
    amount.resize(rows, std::vector<int>(cols, 0));
    visited.resize(rows, std::vector<bool>(cols, false));
}
void ImageProcessor::reset()
{
    const int cols = 640;
    const int rows = 480;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            amount[i][j] = 0;
            visited[i][j] = false;
        }
    }
}

cv::Mat ImageProcessor::createCustomMask(const cv::Mat &inputImage)
{
    cv::Mat cloneImage = inputImage.clone();
    int maxAmount = 0;
    int maxAmountX = 0, maxAmountY = 0;
    reset();
    for (int i = 0; i < cloneImage.rows; i++)
    {
        for (int j = 0; j < cloneImage.cols; j++)
        {
            if (cloneImage.at<uchar>(i, j) == 255 && !visited[i][j])
            {
                int currentAmount = dfs(cloneImage, i, j);
                if (currentAmount > maxAmount)
                {
                    maxAmount = currentAmount;
                    maxAmountX = i;
                    maxAmountY = j;
                }
            }
        }
    }
    cv::Mat outputImage = cv::Mat::zeros(cloneImage.size(), CV_8UC1);
    // std::cout << "maxAmountX: " << maxAmountX << " maxAmountY: " << maxAmountY << std::endl;
    // std::cout << "maxAmount: " << maxAmount << std::endl;
    // std::cout << "\033[31m" << "maxAmount: " << maxAmount << "\033[0m" << std::endl;
    resetVisited(cloneImage.rows, cloneImage.cols);
    dye(maxAmountX, maxAmountY, cloneImage, outputImage);
    return outputImage;
}

std::pair<double, std::pair<bool, double>> ImageProcessor::liner(const cv::Mat &img, cv::Mat &result)
{
    int rows = img.rows;
    int cols = img.cols;
    std::vector<cv::Point> points;
    cv::findNonZero(img, points);
    if (points.size() < 2)
    {
        return std::make_pair(0, std::make_pair(false, k_last));
    }
    cv::Vec4f fitted_line;
    cv::fitLine(points, fitted_line, cv::DIST_L2, 0, 0.01, 0.01);
    double vx = fitted_line[0], vy = fitted_line[1], x0 = fitted_line[2], y0 = fitted_line[3];
    cv::Point2f point1, point2;
    point1.x = ((0 - y0) * vx / vy + x0);
    point1.y = 0;
    point2.x = ((rows - y0) * vx / vy + x0);
    point2.y = rows;
    cv::line(result, cv::Point(cols / 2, 0), cv::Point(cols / 2, rows), cv::Scalar(0, 0, 255), 2);
    cv::line(result, point1, point2, cv::Scalar(0, 255, 0), 2);
    // 计算拟合直线中心到图像中心的距离
    // double distance = cols / 2 - (rows * vx / vy + x0);
    double distance = cols / 2 - (point1.x + point2.x) / 2;
    // 计算斜率
    if (point2.x - point1.x == 0)
    {
        return std::make_pair(distance, std::make_pair(true, 0));
    }
    double k = -(point2.y - point1.y) / (point2.x - point1.x);
    std::cout << "k: " << k << std::endl;
    k_last = k;
    return std::make_pair(distance, std::make_pair(false, k));
}

int ImageProcessor::dfs(cv::Mat &img, short x, short y)
{
    if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
        return 0;
    visited[x][y] = true;
    amount[x][y] = 1;
    amount[x][y] += dfs(img, x + 1, y);
    amount[x][y] += dfs(img, x - 1, y);
    amount[x][y] += dfs(img, x, y + 1);
    amount[x][y] += dfs(img, x, y - 1);
    return amount[x][y];
}
void ImageProcessor::dye(short x, short y, cv::Mat &img, cv::Mat &mask)
{
    if (x < 0 || x >= img.rows || y < 0 || y >= img.cols || visited[x][y] || img.at<uchar>(x, y) != 255)
        return;
    visited[x][y] = true;
    mask.at<uchar>(x, y) = 255;
    dye(x + 1, y, img, mask);
    dye(x - 1, y, img, mask);
    dye(x, y + 1, img, mask);
    dye(x, y - 1, img, mask);
}

void ImageProcessor::resetVisited(int rows, int cols)
{
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            visited[i][j] = false;
        }
    }
}

void LineProcessor::preTreat(cv::Mat img,
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
void LineProcessor::lineDetect(cv::Mat &img,
                               std::vector<cv::Vec4i> &lines, cv::Mat &mask0Color, cv::Mat &mask1Color, cv::Mat &edgeColor)
{
    // 图像预处理
    cv::Mat edges;
    preTreat(img, maskBGRGenerator, imageProcessor,
             mask0Color, mask1Color, edges);
    // 使用霍夫变换检测直线
    const int linelength = 100;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, linelength, 25);
    // 在彩色掩膜上绘制检测到的直线
    cv::cvtColor(edges, edgeColor, cv::COLOR_GRAY2BGR); // 颜色阈值掩膜二值图转彩色图
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(edgeColor, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}
void LineProcessor::lineCount(const std::vector<cv::Vec4i> &lines,
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
void LineProcessor::findLineEndsByDegree(const int &degree, const std::vector<cv::Vec4i> &lines,
                                         cv::Point &leftPoint, cv::Point &rightPoint)
{
    if (degree >= 0)
    {
        cv::Point leftButtomPoint(INT_MAX, 0);
        cv::Point rightTopPoint(0, INT_MAX);
        for (const auto &line : lines)
        {
            cv::Point p1(line[0], line[1]);
            cv::Point p2(line[2], line[3]);
            leftButtomPoint.x = std::min(leftButtomPoint.x, p1.x);
            leftButtomPoint.x = std::min(leftButtomPoint.x, p2.x);
            leftButtomPoint.y = std::max(leftButtomPoint.y, p1.y);
            leftButtomPoint.y = std::max(leftButtomPoint.y, p2.y);
            rightTopPoint.x = std::max(rightTopPoint.x, p1.x);
            rightTopPoint.x = std::max(rightTopPoint.x, p2.x);
            rightTopPoint.y = std::min(rightTopPoint.y, p1.y);
            rightTopPoint.y = std::min(rightTopPoint.y, p2.y);
        }
        leftPoint = leftButtomPoint;
        rightPoint = rightTopPoint;
    }
    else
    {
        cv::Point leftTopPoint(INT_MAX, INT_MAX);
        cv::Point rightButtomPoint(0, 0);
        for (const auto &line : lines)
        {
            cv::Point p1(line[0], line[1]);
            cv::Point p2(line[2], line[3]);
            leftTopPoint.x = std::min(leftTopPoint.x, p1.x);
            leftTopPoint.x = std::min(leftTopPoint.x, p2.x);
            leftTopPoint.y = std::min(leftTopPoint.y, p1.y);
            leftTopPoint.y = std::min(leftTopPoint.y, p2.y);
            rightButtomPoint.x = std::max(rightButtomPoint.x, p1.x);
            rightButtomPoint.x = std::max(rightButtomPoint.x, p2.x);
            rightButtomPoint.y = std::max(rightButtomPoint.y, p1.y);
            rightButtomPoint.y = std::max(rightButtomPoint.y, p2.y);
        }
        leftPoint = leftTopPoint;
        rightPoint = rightButtomPoint;
    }
}

void LineProcessor::lineFit(cv::Mat &img,
                            cv::Point &midPoint, int &degree)
{
    cv::Mat mask0Color, mask1Color, edgeColor;
    // 检测直线
    std::vector<cv::Vec4i> lines;
    lineDetect(img, lines, mask0Color, mask1Color, edgeColor);
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
    // 获取数量最多的角度
    degree = degreeMaxCount;
    if (degree < 0)
        degree += 180;
    std::vector<cv::Vec4i> linesMost = linesByDegree[degreeMaxCount];
    cv::Point leftPoint, rightPoint;
    findLineEndsByDegree(degreeMaxCount, linesMost, leftPoint, rightPoint);
    // cv::Point degree
    // 计算中点
    midPoint.x = (leftPoint.x + rightPoint.x) / 2;
    midPoint.y = (leftPoint.y + rightPoint.y) / 2;
    // midPoint((leftPoint.x + rightPoint.x) / 2, (leftPoint.y + rightPoint.y) / 2);
    // 以degreeMaxCount作为角度，midPoint作为中点，绘制一条直线
    double radian = degreeMaxCount * (CV_PI / 180.0);
    double dx = std::cos(radian);
    double dy = std::sin(radian);
    cv::Point p1(midPoint.x - 1000 * dx, midPoint.y - 1000 * dy);
    cv::Point p2(midPoint.x + 1000 * dx, midPoint.y + 1000 * dy);
    cv::line(edgeColor, p1, p2, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    lineShow(img, mask0Color, mask1Color, edgeColor);
}
void LineProcessor::lineShow(cv::Mat &img, cv::Mat &mask0Color, cv::Mat &mask1Color, cv::Mat &edgeColor)
{
    {
        // 水平拼接 img 和 mask0Color
        cv::Mat horizontal1;
        cv::hconcat(img, mask0Color, horizontal1);
        // 水平拼接 mask1Color 和 edgeColor
        cv::Mat horizontal2;
        cv::hconcat(mask1Color, edgeColor, horizontal2);
        // 垂直拼接两个水平拼接得到的图像
        cv::Mat finalImage;
        cv::vconcat(horizontal1, horizontal2, finalImage);
        // 显示最终的图像
        cv::imshow("Final Image", finalImage);
    }
}