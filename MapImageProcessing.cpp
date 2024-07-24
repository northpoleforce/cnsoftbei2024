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

std::pair<cv::Point, cv::Point> ImageProcessor::findLineEndsByDegree(const int &degree, const std::vector<cv::Vec4i> &lines)
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
        return std::make_pair(leftButtomPoint, rightTopPoint);
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
        return std::make_pair(leftTopPoint, rightButtomPoint);
    }
}