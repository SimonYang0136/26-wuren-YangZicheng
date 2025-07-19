aux_source_directory(src A2_SOURCES)

# 手动添加源文件以防auto没有找到
if(NOT A2_SOURCES)
    set(A2_SOURCES src/A2.cpp)
endif()

add_library(A2 STATIC ${A2_SOURCES})

target_include_directories(A2 PUBLIC include)
