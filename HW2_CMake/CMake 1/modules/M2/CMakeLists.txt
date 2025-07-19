aux_source_directory(src M2_SOURCES)

add_library(M2 STATIC ${M2_SOURCES})

target_include_directories(M2 
    PUBLIC include
    PRIVATE 
    ../../common/kalman/include
    ../A1/include
    ../A2/include
)
target_link_libraries(M2 PUBLIC A1 A2 kalman)
