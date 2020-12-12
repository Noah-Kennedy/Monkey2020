find_package(zed)

file(
        GLOB SOURCES_LIST CONFIGURE_DEPENDS
        src/*.cpp
)

add_library(visual-processing-cpp ${SOURCES_LIST})

target_include_directories(
        visual-processing-cpp
        PUBLIC include
        PRIVATE private-include
)

if (zed_FOUND)
    message(STATUS "Found ZED SDK")

    target_link_libraries(
            visual-processing-cpp
            PRIVATE zed
    )
else ()
    message(WARNING "ZED SDK not found")
    target_link_libraries(
            zed-header-only
            PRIVATE zed
    )
endif ()