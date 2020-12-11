# Works with 3.11 and tested through 3.19
cmake_minimum_required(VERSION 3.12)

# Project name and a few useful settings. Other commands can pick up the results
project(
        monkey-cpp
        LANGUAGES CXX
)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif ()

# The compiled library code is here
add_subdirectory(visual-processing-cpp)