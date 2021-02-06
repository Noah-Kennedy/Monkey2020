find_package(zed 3)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_executable(visual-processing-cpp ${SOURCES_LIST})

target_include_directories(
        visual-processing-cpp
        PUBLIC include
        PRIVATE private-include
)

    message(STATUS "Found ZED SDK")

    target_link_libraries(
            visual-processing-cpp
            PRIVATE zed
	    PRIVATE ${ZED_LIBRARIES}
    )

    target_include_directories(
            visual-processing-cpp
	    PRIVATE ${ZED_INCLUDE_DIRS}
    )
    message(STATUS "Found Cuda")

    target_link_libraries(
            visual-processing-cpp
	    PRIVATE cuda
    )
    target_include_directories(
            visual-processing-cpp
	    PRIVATE ${CUDA_INCLUDE_DIRS}
	    PRIVATE ${CUDA_LIBRARY_DIR}
    )


    message(STATUS "Found OpenCV")

    target_link_libraries(
            visual-processing-cpp
            PRIVATE OpenCV
    )
    message(STATUS "Found apriltag")

    target_link_libraries(
            visual-processing-cpp
            PRIVATE apriltag
	    )
