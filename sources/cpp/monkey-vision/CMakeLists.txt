find_package(OpenCV REQUIRED)
find_package(zed 3)

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_executable(visual-processing-cpp ${SOURCES_LIST})

target_include_directories(
        visual-processing-cpp
        PUBLIC include
        PRIVATE private-include
        PRIVATE ${ZED_INCLUDE_DIRS}
        PRIVATE ${OpenCV_INCLUDE_DIRS}
        PRIVATE ${CUDA_INCLUDE_DIRS}
)

message(STATUS "Found ZED SDK")

target_link_libraries(
        visual-processing-cpp
        PRIVATE ${ZED_LIBRARIES}
        PRIVATE ${CUDA_LIBRARIES}
        PRIVATE ${OpenCV_LIBS}
        PRIVATE ${CUDA_npps_LIBRARY}
)
