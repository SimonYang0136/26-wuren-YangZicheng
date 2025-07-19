cmake_minimum_required(VERSION 3.10)
project(Test LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 添加子目录
add_subdirectory("CMake 1/modules/A1")
add_subdirectory("CMake 1/modules/A2")
add_subdirectory("CMake 1/modules/M1")
add_subdirectory("CMake 1/modules/M2")
add_subdirectory("CMake 1/common/math")
add_subdirectory("CMake 1/common/kalman")

# 创建test可执行文件
add_executable(test "CMake 1/main.cpp")

# main.cpp包含了M1，M2和math的头文件
target_include_directories(test PRIVATE 
    "CMake 1/modules/M1/include"
    "CMake 1/modules/M2/include"
    "CMake 1/common/math/include"
)

#main.cpp依赖M1,M2,math,opencv库
target_link_libraries(test M1 M2 math ${OpenCV_LIBS})