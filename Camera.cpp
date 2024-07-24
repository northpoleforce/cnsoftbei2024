#include "Camera.h"
#include <iostream>

Camera::Camera(int cam_id, int width, int height)
    : cam_id(cam_id), width(width), height(height)
{
    std::string ip_last_segment = "123";
    std::string udpstr_prev_data = "udpsrc address=192.168.123." + ip_last_segment + " port=";
    std::vector<int> udp_port = {9201, 9202, 9203, 9204, 9205};
    std::string udpstr_behind_data = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
    std::cout << udp_send_integrated_pipe_0 << std::endl;
    cap.open(udp_send_integrated_pipe_0);
}

Camera::~Camera()
{
    cv::destroyAllWindows();
    cap.release();
}

void Camera::reopen()
{
    cap.release();
    cap.open(udp_send_integrated_pipe_0);
}

cv::Mat Camera::getFrame()
{
    cv::Mat frame;
    while (!cap.read(frame))
        ;
    cv::resize(frame, frame, cv::Size(width, height));
    if (cam_id == 1)
    {
        cv::flip(frame, frame, -1);
    }
    cv::waitKey(1);
    return frame;
}