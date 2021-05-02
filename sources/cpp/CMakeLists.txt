cmake_minimum_required(VERSION 3.12)

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

add_subdirectory(zed)
add_subdirectory(monkey-vision)