#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/gpu/gpu.hpp>
// #include <opencv2/cudaimgproc.hpp>

#include <Camera.h>

int main()
{
    std::cout << cv::cuda::getCudaEnabledDeviceCount() << std::endl;
    // while (1)
    // {
    //     Camera cam(2);
    //     cv::Mat frame = cam.getFrame();
    //     cv::imshow("frame", frame);
    //     cv::waitKey(2);
    //     cv::Mat src = frame;
    //     // 创建一个GPU矩阵并上传图像
    //     cv::gpu::GpuMat d_src(src);
    //     // 创建一个GPU矩阵来存储结果
    //     cv::gpu::GpuMat d_dst;
    //     // 使用GPU进行Canny边缘检测
    //     cv::gpu::Canny(d_src, d_dst, 50, 150);
    //     // 下载结果到主机内存
    //     cv::Mat dst;
    //     d_dst.download(dst);

    //     // 显示结果
    //     cv::imshow("Edges", dst);
    //     cv::waitKey(2);
    // }

    return 0;
}