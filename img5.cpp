#include <Python.h>
#include <iostream>
#include <string>
#include <array>
#include <cstdlib>

#define DEBUG_ARM

// const std::string py_12L = "../1-2(left).py";
// const std::string py_12R = "../1-2(left).py";
// const std::string py_13L = "../1-3(left).py";
// const std::string py_13R = "../1-3(left).py";
// const std::string py_14L = "../1-4(left).py";
// const std::string py_14R = "../1-4(left).py";
// const std::string py_23L = "../2-3(left).py";
// const std::string py_23R = "../2-3(left).py";
// const std::string py_24L = "../2-4(left).py";
// const std::string py_24R = "../2-4(left).py";
// const std::string py_34L = "../3-4(left).py";
// const std::string py_34R = "../3-4(left).py";

std::array<std::array<std::array<std::string, 2>, 4>, 4> py_files = {{{{{"../test/1-2(left).py", "../test/1-2(right).py"},
                                                                        {"../test/1-3(left).py", "../test/1-3(right).py"},
                                                                        {"../test/1-4(left).py", "../test/1-4(right).py"},
                                                                        {"", ""}}},
                                                                      {{{"", ""},
                                                                        {"../test/2-3(left).py", "../test/2-3(right).py"},
                                                                        {"../test/2-4(left).py", "../test/2-4(right).py"},
                                                                        {"", ""}}},
                                                                      {{{"", ""},
                                                                        {"", ""},
                                                                        {"../test/3-4(left).py", "../test/3-4(right).py"},
                                                                        {"", ""}}},
                                                                      {{{"", ""},
                                                                        {"", ""},
                                                                        {"", ""},
                                                                        {"", ""}}}}};

int main()
{
    // // for (const auto &group : py_files)
    // // {
    // //     for (const auto &pair : group)
    // //     {
    // //         for (const auto &file : pair)
    // //         {
    // //             std::string command = "python3 " + file;
    // //             std::system(command.c_str());
    // //         }
    // //     }
    // // }
    // std::string command = "python3 " + py_files[1][2][0];
    // std::system(command.c_str());
    // for (int i = 0; i < 4; i++)
    // {
    //     for (int j = 0; j < 4; j++)
    //     {
    //         for (int k = 0; k < 2; k++)
    //         {
    //             if (py_files[i][j][k] != "")
    //             {
    //                 std::cout << py_files[i][j][k] << std::endl;
    //             }
    //         }
    //     }
    // }
    FILE* file;

    Py_Initialize();  // 初始化Python环境

    // file = fopen("../hi.py", "r");  // 打开Python脚本
    // PyRun_SimpleFile(file, "../hi.py");  // 执行Python脚本
    std::cout << py_files[1][2][0] << std::endl;
    file = fopen(py_files[1][2][0].c_str(), "r");  // 打开Python脚本
    if (file != nullptr) {
        PyRun_SimpleFile(file, py_files[1][2][0].c_str());
        fclose(file);  // 关闭文件
    } else {
        // 打开文件失败，处理错误
    }
    // PyRun_SimpleFile(file, py_files[1][2][0].c_str());  // 执行Python脚本

    Py_Finalize();  // 清理并关闭Python环境

    return 0;
}