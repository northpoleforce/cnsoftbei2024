#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480);
    ~Camera();
    void reopen();
    cv::Mat getFrame();

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
    std::string udp_send_integrated_pipe_0;
};