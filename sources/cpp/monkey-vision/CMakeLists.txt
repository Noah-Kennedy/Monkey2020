find_package(OpenCV REQUIRED COMPONENTS aruco)
find_package(zed 3 REQUIRED)

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_library(monkey-vision ${SOURCES_LIST})

set_target_properties(
        monkey-vision
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
)

install(TARGETS monkey-vision)

target_include_directories(
        monkey-vision
        PUBLIC include
        PRIVATE ${ZED_INCLUDE_DIRS}
        PRIVATE ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(
        monkey-vision
        PUBLIC cameralot
        PRIVATE opencv_aruco
        PRIVATE ${ZED_LIBRARIES}
)