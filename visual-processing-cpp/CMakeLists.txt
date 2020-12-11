file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_library(visual-processing-cpp ${SOURCES_LIST})

target_include_directories(
        visual-processing-cpp
        PUBLIC include
)