set(SOURCES_HEADLESS
    main.cpp
    Platform.cpp
)

add_executable(melonDS-headless ${SOURCES_HEADLESS})
target_link_libraries(melonDS-headless PRIVATE core)

find_package(Threads REQUIRED)
target_link_libraries(melonDS-headless PRIVATE Threads::Threads)

include(FetchContent)
FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG        865adac
)
FetchContent_MakeAvailable(lodepng)
if (NOT TARGET lodepng)
    add_library(lodepng STATIC ${lodepng_SOURCE_DIR}/lodepng.cpp)
endif()
target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})

target_link_libraries(melonDS-headless PRIVATE lodepng)

if (CMAKE_DL_LIBS)
    target_link_libraries(melonDS-headless PRIVATE ${CMAKE_DL_LIBS})
endif()
