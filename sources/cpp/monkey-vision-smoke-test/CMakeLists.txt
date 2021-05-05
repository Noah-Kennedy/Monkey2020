find_package(OpenCV REQUIRED)
find_package(zed 3 REQUIRED)

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_executable(monkey-vision-smoke ${SOURCES_LIST})

target_include_directories(
        monkey-vision-smoke
        PRIVATE ${ZED_INCLUDE_DIRS}
        PRIVATE ${OpenCV_INCLUDE_DIRS}
        PRIVATE ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(
        monkey-vision-smoke
        PRIVATE monkey-vision
        PRIVATE ${ZED_LIBRARIES}
        PRIVATE ${CUDA_LIBRARIES}
        PRIVATE ${OpenCV_LIBS}
        PRIVATE ${CUDA_npps_LIBRARY}
)