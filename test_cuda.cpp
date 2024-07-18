#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/gpu/gpu.hpp>
#include <opencv2/cudaimgproc.hpp>

class Camera
{
public:
    Camera(int cam_id = 0, int width = 640, int height = 480)
        : cam_id(cam_id), width(width), height(height)
    {
        std::string ip_last_segment = "123";
        std::string udpstr_prev_data = "udpsrc address=192.168.123." + ip_last_segment + " port=";
        std::vector<int> udp_port = {9201, 9202, 9203, 9204, 9205};
        std::string udpstr_behind_data = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
        std::string udp_send_integrated_pipe_0 = udpstr_prev_data + std::to_string(udp_port[cam_id - 1]) + udpstr_behind_data;
        std::cout << udp_send_integrated_pipe_0 << std::endl;
        cap.open(udp_send_integrated_pipe_0);
    }
    ~Camera()
    {
        cv::destroyAllWindows();
        cap.release();
    }
    cv::Mat getFrame()
    {
        cv::Mat frame;
        while (!cap.read(frame))
            ;
        cv::resize(frame, frame, cv::Size(width, height));
        if (cam_id == 1)
        {
            cv::flip(frame, frame, -1);
        }
        cv::imshow("original", frame);
        if (cv::waitKey(2) == 'q')
        {
            cv::destroyAllWindows();
            exit(0);
        }
        return frame;
    }

private:
    int cam_id;
    int width;
    int height;
    cv::VideoCapture cap;
};

int main()
{
    while (1)
    {
        Camera cam(2);
        cv::Mat frame = cam.getFrame();
        cv::imshow("frame", frame);
        cv::waitKey(2);
        cv::Mat src = frame;

        // 创建一个GPU矩阵并上传图像
        cv::gpu::GpuMat d_src(src);

        // 创建一个GPU矩阵来存储结果
        cv::gpu::GpuMat d_dst;

        // 使用GPU进行Canny边缘检测
        cv::gpu::Canny(d_src, d_dst, 50, 150);

        // 下载结果到主机内存
        cv::Mat dst;
        d_dst.download(dst);

        // 显示结果
        cv::imshow("Edges", dst);
        cv::waitKey(2);
    }

    return 0;
}