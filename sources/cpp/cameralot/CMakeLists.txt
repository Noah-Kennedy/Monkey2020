find_package(OpenCV REQUIRED COMPONENTS core videoio)
include_directories(${OpenCV_INCLUDE_DIRS})

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_library(cameralot-capture ${SOURCES_LIST})
install(TARGETS cameralot-capture)

target_include_directories(
        cameralot-capture
        PUBLIC include
        PRIVATE opencv_core opencv_video opencv_imgproc
)

# OpenCV is required; link it here
target_link_libraries(
        cameralot-capture ${OpenCV_LIBS}
)