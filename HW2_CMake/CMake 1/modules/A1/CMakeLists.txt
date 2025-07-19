aux_source_directory(src A1_SOURCES)

add_library(A1 STATIC ${A1_SOURCES})#小型模块，使用静态库

target_include_directories(A1 PUBLIC include)
# PUBLIC: 其他目标在链接A1时可以找到A1的头文件
