#include <opencv2/opencv.hpp>

#include <Camera.h>

int main() {
    Camera cam2(2);

    // 获取默认摄像头的帧率
    int frame_width = 640;
    int frame_height = 480;
    int fps = 15;

    // 创建一个 VideoWriter 对象
    cv::VideoWriter video("out.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frame_width, frame_height));

    cv::Mat frame;
    while (1) {
        frame = cam2.getFrame();
        if (frame.empty())
            break;
        video.write(frame);

        // 显示帧
        imshow("Frame", frame);

        // 按 'q' 键退出
        if (cv::waitKey(1) == 'q')
            break;
    }

    video.release();
    // 关闭所有 OpenCV 窗口
    cv::destroyAllWindows();
    return 0;
}