#include "DogVision.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

// 无参构造, 只是打开摄像头
DogVision::DogVision() : cam("../config/stereo_camera_config.yaml") {

}

// 有参构造, 构造函数会读取配置文件，并根据文件内容设置参数值, 并传入控制运动的指针
DogVision::DogVision(const std::string& configFile, DogMotion* myPlan) : cam("../config/stereo_camera_config.yaml") {
    dogMotion = myPlan;
    printf("okk\n");
    std::ifstream config_file(configFile);
    std::string line;
    // read the config file
    while (std::getline(config_file, line)) {
        std::istringstream iss(line);
        std::string name;
        int value;
        char equal_sign;
        if (!(iss >> name >> equal_sign >> value) || equal_sign != '=') {
            throw std::runtime_error("Error parsing config file: " + line);
        }
        // 从配置文件中获取各项参数的值
        if (name == "cannyThreshold") {
            cannyThreshold = value;
        } else if (name == "cannyThresholdMax") {
            cannyThresholdMax = value;
        } else if (name == "houghThreshold") {
            houghThreshold = value;
        } else if (name == "houghMinLineLength") {
            houghMinLineLength = value;
        } else if (name == "houghMaxLineGap") {
            houghMaxLineGap = value;
        } else if (name == "yellowHLower") {
            yellowHLower = value;
        } else if (name == "yellowSLower") {
            yellowSLower = value;
        } else if (name == "yellowVLower") {
            yellowVLower = value;
        } else if (name == "yellowHUpper") {
            yellowHUpper = value;
        } else if (name == "yellowSUpper") {
            yellowSUpper = value;
        } else if (name == "yellowVUpper") {
            yellowVUpper = value;
        } else if (name == "lowerAngel") {
            lowerAngel = value;
        } else if (name == "upperAngel") {
            upperAngel = value;
        } else if (name == "dropNode1") {
            dogMotion->flag1 = value;
        } else if (name == "dropNode2") {
            dogMotion->flag2 = value;
        } else if (name == "dropNode3") {
            dogMotion->flag3 = value;
        } else if (name == "dropNode4") {
            dogMotion->flag4 = value;
        } else {
            throw std::runtime_error("Unknown parameter in config file: " + name);
        }
    }
}


// 这个函数用于处理图像，你需要根据你的具体需求来实现这个函数。
// 检测黄色区域，去掉黄色区域周围的一圈，确定左右边界，计算中心点，根据偏差调整前进
void DogVision::processImage(cv::Mat& src) {
    // 转hsv，中值滤波去噪
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    medianBlur(hsv, hsv, 5);
    // Use the config values fr yoellow color range
    // 黄色阈值掩膜
    Mat mask;
    Scalar yellow_lower = Scalar(yellowHLower, yellowSLower, yellowVLower);
    Scalar yellow_upper = Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    inRange(hsv, yellow_lower, yellow_upper, mask);

    cv::imshow("original_mask", mask);

    // Create a structuring element (you may need to adjust the size depending on your image)
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));

    Mat after_mask;
    // Perform morphological opening
    cv::morphologyEx(mask, after_mask, cv::MORPH_OPEN, structuringElement);
    cv::imshow("after_mask", after_mask);

    Mat road;
    // road是原图中的黄色区域
    bitwise_and(src, src, road, mask);
    // road设置为紫色
    road.setTo(Scalar(255, 0, 255), mask);
    // road和src叠加，高亮原来的黄色区域
    addWeighted(src, 1, road, 1, 0, src);
    // 把mask的中黄色区域的周围一圈设置为0

    cv::imshow("src", src);

    // UP
    // 左上角(列，高)，截取的面积(列，高)
    mask(Rect(0, 0, mask.cols, mask.rows * 2 / 7)) = 0; // UP
    // 上面截掉3/7
    // double scale_up = 3.0/7; mask(Rect(0, 0, mask.cols, mask.rows*scale_up)) = 0; // UP
    double scale_left = 2.0/7; mask(Rect(0, mask.rows * 2 / 7, mask.cols / 7, mask.rows * 4 / 7)) = 0; // left
    double sacle_right = 1/7; mask(Rect(mask.cols * 6 / 7, mask.rows * 2 / 7, mask.cols / 7, mask.rows * 4 / 7)) = 0; // right
    mask(Rect(0, mask.rows * 6 / 7, mask.cols, mask.rows * 1 / 7 + 1)) = 0; // beneath
    // 下面
    // double scale_buttom = 2.0/7; 
    // mask(Rect(0, mask.rows*(1-scale_buttom), mask.cols, mask.rows*scale_buttom+1)) = 0; // bottom

    // cv::imshow("mask", mask);

    Mat edges;
    // 对mask进行边缘检测，保存在edges中
    // 使用两个阈值（minVal 和 maxVal）。如果图像的灰度梯度大于 maxVal，则被认为是“真正的边缘”。如果小于 minVal，则被认为是“非边缘”。
    // 如果介于这两者之间，那么如果该像素连接到“真正的边缘”像素，则认为它是边缘的一部分，否则，它也被认为是“非边缘”。
    Canny(mask, edges, cannyThreshold, cannyThresholdMax);

    cv::imshow("edges", edges);

    cv::Mat color_edges;
    // Hough 变换检测直线
    // edges 是输入的边缘图像，lines 是输出的线段，1 和 CV_PI / 180 是 rho 和 theta 的分辨率，
    // houghThreshold 是判断直线点数的阈值，houghMinLineLength 是线段的最小长度，houghMaxLineGap 是同一条线段上两点之间的最大距离
    // Vec4i 是一个包含四个整数的向量，对于每条检测到的线，它包含线的两个端点的坐标 (x1, y1, x2, y2)
    

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
    cv::cvtColor(edges, color_edges, cv::COLOR_GRAY2BGR);
    for(size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(color_edges, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    // std::vector<cv::Vec2f> lines;
    // cv::HoughLines(edges, lines, 1, CV_PI/180, 100, 0, 0, CV_PI/2-CV_PI/180, CV_PI/2+CV_PI/180);
    // Draw the lines
    // for(size_t i = 0; i < lines.size(); i++)
    // {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     cv::Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     cv::line(color_edges, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    // }
    cv::imshow("color_edges", color_edges);


    // // Filter and classify lines
    vector<Vec4i> left_lines, right_lines;
    for (auto line: lines) {
        // 计算每条直线的角度。这里使用 atan2 函数计算两点之间的角度，然后将其转换为度数
        double angle = abs(atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI);
        //cout << "angle: " << angle << endl;
        // 如果直线的角度在 lowerAngel 和 upperAngel 之间
        if ((angle >= lowerAngel) && (angle < upperAngel)) {
            // 如果直线的 x 坐标小于图像宽度的一半，那么将这条直线分类到 left_lines 中，否则分类到 right_lines 中
            if (line[2] < src.cols / 2) {
                left_lines.push_back(line);
            } else {
                right_lines.push_back(line);
            }
        }
    }
    // Draw the line of direction
    // 图像上绘制一条垂直线的命令。这条线从图像的顶部到底部，x 坐标为图像宽度的一半。这条线的颜色为绿色（RGB 值为 (0, 255, 0)），线宽为 2。
    line(src, Point(src.cols / 2, 0), Point(src.cols / 2, src.rows), Scalar(0, 255, 0), 2);

    cv::imshow("line_src", src);

    Point left_center(0, 0), right_center(0, 0);
    for (auto line: left_lines) {
        left_center.x += (line[0] + line[2]) / 2;
        left_center.y += (line[1] + line[3]) / 2;
    }
    for (auto line: right_lines) {
        right_center.x += (line[0] + line[2]) / 2;
        right_center.y += (line[1] + line[3]) / 2;
    }
    left_center.x /= left_lines.size();
    left_center.y /= left_lines.size();
    right_center.x /= right_lines.size();
    right_center.y /= right_lines.size();

    // Calculate the average slope and midpoint of the left lines
    double left_slope = 0;
    cv::Point left_midpoint(0, 0);
    for(const auto& line : left_lines)
    {
        left_slope += static_cast<double>(line[3] - line[1]) / (line[2] - line[0]);
        left_midpoint.x += (line[0] + line[2]) / 2;
        left_midpoint.y += (line[1] + line[3]) / 2;
    }
    left_slope /= left_lines.size();
    left_midpoint.x /= left_lines.size();
    left_midpoint.y /= left_lines.size();

    // Calculate the average slope and midpoint of the right lines
    double right_slope = 0;
    cv::Point right_midpoint(0, 0);
    for(const auto& line : right_lines)
    {
        right_slope += static_cast<double>(line[3] - line[1]) / (line[2] - line[0]);
        right_midpoint.x += (line[0] + line[2]) / 2;
        right_midpoint.y += (line[1] + line[3]) / 2;
    }
    right_slope /= right_lines.size();
    right_midpoint.x /= right_lines.size();
    right_midpoint.y /= right_lines.size();

    // Calculate the average slope and midpoint of the center line
    double center_slope = (left_slope + right_slope) / 2;
    cv::Point center_midpoint((left_midpoint.x + right_midpoint.x) / 2, (left_midpoint.y + right_midpoint.y) / 2);

    // Create the center line
    cv::Vec4i center_line;
    center_line[0] = center_midpoint.x - 1000; // x1
    center_line[1] = center_midpoint.y - 1000 * center_slope; // y1
    center_line[2] = center_midpoint.x + 1000; // x2
    center_line[3] = center_midpoint.y + 1000 * center_slope; // y2

    // for(size_t i = 0; i < lines.size(); i++)
    {
        // cv::Vec4i l = lines[i];
        cv::line(src, cv::Point(center_line[0], center_line[1]), cv::Point(center_line[2], center_line[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    // Calculate and draw the center point
    // 左右边界的中心点的中心点
    Point center_point((left_center.x + right_center.x) / 2, (left_center.y + right_center.y) / 2);
    // 绘制中心点：圆心为 center_point，半径为 5，颜色为红色（RGB 值为 (0, 0, 255)），填充圆（线宽为 -1）
    circle(src, center_point, 5, Scalar(0, 0, 255), -1);

    cv::imshow("final_src", src);

    // Create a new blank image
    cv::Mat blank = cv::Mat::zeros(src.size(), src.type());
    // Draw a circle on the blank image
    cv::circle(blank, center_point, 5, cv::Scalar(0, 0, 255), -1);
    // cv::imshow("circle", blank);


    // // Check if we have both left and right lines
    // if (left_lines.empty() || right_lines.empty()) {
    //     // 如果左右两边没有一起检测到直线
    //     //cout << "Can't find two boundary" << endl;
    //     // Mat merge;
    //     // hconcat(mask, edges, merge);
    //     // imshow("Detected Lines", src);
    //     // imshow("mask edges", merge);
    //     //waitKey(1);
    // } else {
    //     // Calculate the average center point of all left and right lines
    //     // 计算左右边界的中心点：坐标和除以数量
    //     Point left_center(0, 0), right_center(0, 0);
    //     for (auto line: left_lines) {
    //         left_center.x += (line[0] + line[2]) / 2;
    //         left_center.y += (line[1] + line[3]) / 2;
    //     }
    //     for (auto line: right_lines) {
    //         right_center.x += (line[0] + line[2]) / 2;
    //         right_center.y += (line[1] + line[3]) / 2;
    //     }
    //     left_center.x /= left_lines.size();
    //     left_center.y /= left_lines.size();
    //     right_center.x /= right_lines.size();
    //     right_center.y /= right_lines.size();

    //     // Calculate and draw the center point
    //     // 左右边界的中心点的中心点
    //     Point center_point((left_center.x + right_center.x) / 2, (left_center.y + right_center.y) / 2);
    //     // 绘制中心点：圆心为 center_point，半径为 5，颜色为红色（RGB 值为 (0, 0, 255)），填充圆（线宽为 -1）
    //     circle(src, center_point, 5, Scalar(0, 0, 255), -1);

    //     // Create a new blank image
    //     cv::Mat blank = cv::Mat::zeros(src.size(), src.type());
    //     // Draw a circle on the blank image
    //     cv::circle(blank, center_point, 5, cv::Scalar(0, 0, 255), -1);

    //     cv::imshow("final_src", src);
    //     cv::imshow("circle", blank);

    //     // use motiontime to control:
    //     // 如果中心点的 x 坐标小于图像宽度的一半减 20，说明中心点偏左，需要向右调整 且 在黄色区域内
    //     if (center_point.x < (src.cols / 2 - 20) && !dogMotion->yellowLeft && !dogMotion->yellowRight) { // delete isTime()
    //         cout << "Mid Correcting Right ! !" << endl;
    //         dogMotion->isVision = true;
    //         // 在 黄色区域内 向右调整
    //         dogMotion->midShiftRight = true;
    //         sleep(1);
    //         dogMotion->midShiftRight = false;
    //         dogMotion->isVision = false;
    //     }
    //     if (center_point.x > (src.cols / 2 + 20) && !dogMotion->yellowLeft && !dogMotion->yellowRight) {  // delete isTime()
    //         cout << " Mid Correcting Left ! !" << endl;
    //         // dogMotion->isVision = true;
    //         dogMotion->isVision = true;
    //         dogMotion->midShiftLeft = true;
    //         // 在 黄色区域内 向左调整
    //         sleep(1);
    //         dogMotion->midShiftLeft = false;
    //         dogMotion->isVision = false;
    //         // Mat merge;
    //         // hconcat(mask, edges, merge);
    //         // imshow("Detected Lines", src);
    //         // imshow("mask edges", merge);
    //         // imshow("menubur", temp);
    //         //waitKey(1);
    //     }
    // }
}

// 检测黄色区域，把图像分成左右两部分，计算左右两部分的黄色区域的像素数量，如果左边的黄色区域的像素数量多，那么向左调整，反之向右调整
// 调整为狗左右平移
void DogVision::detectYellow(cv::Mat& image)
{
    //预处理图像
    Mat src = image;
    int rows = image.rows;
    int cols = image.cols;
    // 左边界 left 是图像宽度的 1/5，右边界 right 是图像宽度的 4/5
    int left = cols / 5;
    int right = cols * 4 / 5;
    // 创建一个 cv::Rect 对象 roi，表示裁剪区域。裁剪区域的宽度是 right - left，高度是 rows
    Rect roi(left, 0, right - left, rows);
    // 使用 roi 裁剪 image，并将结果赋值给 image
    image = image(roi);

    /*int top = rows / 4;
    int bottom = rows * 3 / 4;
    Rect roi(0, top, cols, bottom - top);
    image = image(roi);*/

    // 对 image 进行中值滤波，滤波器的大小是 5
    // 去除图像中的椒盐噪声
    medianBlur(image, image, 5);

    // 创建一个掩膜，只显示黄色区域
    // HSV 是一种颜色模型，全称为 Hue（色调）、Saturation（饱和度）和 Value（亮度）
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    // 创建两个 Scalar 对象 lowerYellow 和 upperYellow，分别表示黄色的 HSV 值的下限和上限
    // 构造两个类实例，分别表示黄色的 HSV 值的下限和上限
    Scalar lowerYellow = Scalar(yellowHLower, yellowSLower, yellowVLower);
    Scalar upperYellow = Scalar(yellowHUpper, yellowSUpper, yellowVUpper);
    // 检查 hsv 中的每个像素，如果其值在 lowerYellow 和 upperYellow 之间，则 yellowMask 对应的像素设置为 255（白色），否则设置为 0（黑色）
    Mat yellowMask;
    inRange(hsv, lowerYellow, upperYellow, yellowMask);

    // 在原图截取黄色道路
    // 使用 yellowMask 对原图像进行位与操作，结果存储在 road 中。这样，road 中只包含原图像中的黄色区域，其他区域都是黑色的。
    Mat road;
    bitwise_and(image, image, road, yellowMask);

    //显示图像
    //imshow("image", image);
    //imshow("road",road);
    //imshow("src", src);

    //判断越界
    string direction = "";
    // 黄色区域的像素数量
    int nonZeroCount = countNonZero(yellowMask);
    // 黄色区域的像素数量占整个图像的比例
    double currentRatio = 1.0 * nonZeroCount / (yellowMask.rows * yellowMask.cols);
    double normalRatio = 0.45;
    // 如果黄色区域的像素数量占整个图像的比例小于 normalRatio，说明黄色区域的面积较小，需要调整方向
    if (currentRatio < normalRatio)
    {
        // 分别计算左半部分和右半部分的黄色区域的像素数量
        Mat leftMask = yellowMask(Rect(0, 0, image.cols / 2, image.rows));
        Mat rightMask = yellowMask(Rect(image.cols / 2, 0, image.cols / 2, image.rows));
        int leftCount = countNonZero(leftMask);
        int rightCount = countNonZero(rightMask);

        // control with motiontime and condition
        // 选择黄色区域较多的一侧
        // 对应修改yellowLeft和yellowRight的值，实现方向控制
        if (leftCount < rightCount) // delete isTime()
        {
            dogMotion->isVision = true;
            direction = "left";
            dogMotion->yellowLeft = true;
            sleep(1);
            dogMotion->yellowLeft = false;
            dogMotion->isVision = false;
            cout << "Area: " << direction << endl;
        }
        if (leftCount > rightCount) // delete isTime()
        {
            dogMotion->isVision = true;
            direction = "right";
            dogMotion->yellowRight = true;
            sleep(1);
            dogMotion->yellowRight = false;
            dogMotion->isVision = false;
            cout << "Area: " << direction << endl;
        }
    }
    // else
    // {
    //     //direction = "normal";
    // }
}

void DogVision::detect(cv::Mat src) {
    // cv::Mat gray;
    // cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    // cv::Mat binary;
    // cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
    // cv::imshow("gray", gray);
}

void DogVision::Start() {
    // 相机硬件参数设置
    int deviceNode = 0; ///< default 0 -> /dev/video0
    cv::Size frameSize(1856, 800); ///< default frame size 1856x800
    int fps = 30; ///< default camera fps: 30
    if (!cam.isOpened())   ///< get camera open state
        exit(EXIT_FAILURE);

    // 获取图像
    // cam 宇树的相机类实例
    cam.setRawFrameSize(frameSize); ///< set camera frame size
    cam.setRawFrameRate(fps);       ///< set camera camera fps
    cam.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); ///< set camera rectify frame size
    cam.startCapture(); ///< disable image h264 encoding and share memory sharing

    usleep(500000);
    while (cam.isOpened()) {
        cv::Mat left, right, film, film_clone;
        // 获得三个图像
        // left, right 矫正后的左右图像，使用经纬度坐标系
        // film 矫正过的左图像，使用鱼眼坐标系
        if (!cam.getRectStereoFrame(left, right, film)) { ///< get rectify left,right frame
            usleep(1000);
            continue;
        }

        // cv::imshow("film", film);
        // detect(film);

        film_clone = film.clone();
        // 在黄色道路内，检测偏差：前进转向调整，使得机器人在黄色道路内运动
        processImage(film);
        // 在黄色道路外，检测偏差：左右平移+前进转向调整，使得机器人在不会在黄色道路外运动
        detectYellow(film_clone);
        char key = cv::waitKey(10);
        if (key == 27) // press ESC key
            break;
    }
    cam.stopCapture();
}


