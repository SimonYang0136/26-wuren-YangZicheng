aux_source_directory(src MATH_SOURCES)

add_library(math STATIC ${MATH_SOURCES})

target_include_directories(math PUBLIC include)
target_link_libraries(math ${OpenCV_LIBS})
