cmake_minimum_required(VERSION 3.10)
project(Test LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 手动创建A2库，因为自动发现有问题
aux_source_directory("CMake 1/modules/A2/src" A2_SOURCES)
add_library(A2 STATIC ${A2_SOURCES})
target_include_directories(A2 PUBLIC "CMake 1/modules/A2/include")

# 添加子目录
add_subdirectory("CMake 1/common")
add_subdirectory("CMake 1/modules")

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