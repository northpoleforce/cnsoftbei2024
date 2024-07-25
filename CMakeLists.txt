# 设置cmake最低版本
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(Go1_2024_0704)

# 打印项目名称
message("Project Name: ${PROJECT_NAME}")

# # 添加路径
# list(APPEND CMAKE_PREFIX_PATH "/usr/include/GL" "/usr/lib/x86_64-linux-gnu")
# message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(GLUT REQUIRED)

# 检查系统架构
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    # 如果系统架构是x86_64，设置ARCH变量为amd64
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    # 如果系统架构是aarch64，设置ARCH变量为arm64
    set(ARCH arm64)
endif()

# 找到Python解释器和
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)


# 查找OpenCV库
find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
    # 如果没有找到OpenCV，打印错误信息并退出
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()
# include_directories(${OpenCV_INCLUDE_DIRS}) # 从DIR改为DIRS
# 基本上使用OpenCV需要的依赖库
# 旧的OpenGL库
set(OpenGL_GL_PREFERENCE LEGACY)
# # 新的OpenGL库
# set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if (NOT OpenGL_FOUND)
    message(FATAL_ERROR "OpenGL Library Not Found")
endif()

# find_package(GLUT REQUIRED)
# if (NOT GLUT_FOUND)
#     message(FATAL_ERROR "GLUT Library Not Found")
# endif()
# set(GLUT_INCLUDE_DIR "/usr/include/GL")
# set(GLUT_LIBRARY "/usr/lib/x86_64-linux-gnu/libglut.so")

find_package(X11 REQUIRED)
if (NOT X11_FOUND)
    message(FATAL_ERROR "X11 Library Not Found")
endif()

# find_package(AprilTag REQUIRED)
# if (NOT AprilTag_FOUND)
#     message(FATAL_ERROR "AprilTag Library Not Found")
# endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加头文件目录，这样在编译时可以找到这些头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/unitree_legged_sdk)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/UnitreecameraSDK-main)
include_directories(${AprilTag_INCLUDE_DIRS})
include_directories(/usr/local/include/apriltag)
include_directories(/home/unitree/CAIR/CAIP2024_Go1/include/apriltag)

# 添加链接库目录，这样在链接时可以找到这些库
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/unitree_legged_sdk/${ARCH})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/UnitreecameraSDK-main/${ARCH})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(/usr/local/lib)
link_directories(/home/unitree/CAIR/CAIP2024_Go1/include/apriltag/)

# 设置编译标志：17表示C++17
set(CMAKE_CXX_STANDARD 17)
# 设置编译标志：-O3表示优化等级为3，-fPIC表示生成位置无关代码
set(CMAKE_CXX_FLAGS "-O3 -no-pie")
# 设置编译标志：-pthread表示使用多线程
set(EXTRA_LIBS -pthread libunitree_camera.a libunitree_legged_sdk.a tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${X11_LIBRARIES})

# 添加源文件
# add_executable(main img7.cpp move.cpp)
# add_executable(main test_line.cpp move.cpp)
# add_executable(main test_move.cpp move.cpp)
# add_executable(main test_vision_v0.cpp move.cpp)
# add_executable(main test_cuda.cpp move.cpp Camera.cpp)
# add_executable(main take_photo5.cpp move.cpp Camera.cpp)
# add_executable(main test_video.cpp move.cpp Camera.cpp)
# add_executable(main test_corret.cpp move.cpp Camera.cpp MapImageProcessing.cpp)
# add_executable(main test_corret_v1.cpp move.cpp Camera.cpp MapImageProcessing.cpp)
# add_executable(main test_matchTemplate.cpp move.cpp Camera.cpp)
add_executable(main vision_move_v0.cpp move.cpp Camera.cpp MapImageProcessing.cpp)

# 包含Python头文件
target_include_directories(main PRIVATE ${PYTHON_INCLUDE_DIRS})

# 链接库到可执行文件
target_link_libraries(main ${EXTRA_LIBS} apriltag ${PYTHON_LIBRARIES})