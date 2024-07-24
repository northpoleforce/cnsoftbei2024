#ifndef MAP_IMAGE_PROCESSING_H
#define MAP_IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>

class HSVMaskGenerator
{
public:
    HSVMaskGenerator(int colorHLower, int colorHUpper,
                     int colorSLower, int colorSUpper,
                     int colorVLower, int colorVUpper);
    cv::Mat getMask(cv::Mat src);

private:
    int colorHLower, colorHUpper;
    int colorSLower, colorSUpper;
    int colorVLower, colorVUpper;
};

class BGRMaskGenerator
{
public:
    BGRMaskGenerator(int colorBLower, int colorBUpper,
                     int colorGLower, int colorGUpper,
                     int colorRLower, int colorRUpper);
    cv::Mat getMask(cv::Mat src);

private:
    int colorBLower, colorBUpper;
    int colorGLower, colorGUpper;
    int colorRLower, colorRUpper;
};

class GRAYMaskGenerator
{
public:
    GRAYMaskGenerator(int grayLower, int grayUpper);
    cv::Mat getMask(cv::Mat src);

private:
    int grayLower, grayUpper;
};

class ImageProcessor
{
public:
    ImageProcessor();
    cv::Mat createCustomMask(const cv::Mat &inputImage);
    std::pair<double, std::pair<bool, double>> liner(const cv::Mat &img, cv::Mat &result);
    // 函数：根据线条角度，找线条的两端点；返回值：线条的两端点
    std::pair<cv::Point, cv::Point> findLineEndsByDegree(const int &degree, const std::vector<cv::Vec4i> &lines);

private:
    std::vector<std::vector<int>> amount;
    std::vector<std::vector<bool>> visited;
    double k_last = 0;
    void reset();
    int dfs(cv::Mat &img, short x, short y);
    void dye(short x, short y, cv::Mat &img, cv::Mat &mask);
    void resetVisited(int rows, int cols);
};

#endif // MAP_IMAGE_PROCESSING_H