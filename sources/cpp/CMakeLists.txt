cmake_minimum_required(VERSION 3.18)

project(
        monkey-cpp
        LANGUAGES CXX CUDA
)

# if top level project
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # support IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif ()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD 11)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()

add_subdirectory(monkey-vision)
add_subdirectory(monkey-vision-smoke-test)
add_subdirectory(cameralot)