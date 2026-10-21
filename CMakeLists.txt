cmake_minimum_required(VERSION 3.22)

project(dock_ground_station VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DOCK_GROUND_BUILD_TESTS "Build dock_ground unit tests" ON)
option(DOCK_GROUND_WITH_VRPN "Build the live VRPN pose source" ON)

find_package(Threads REQUIRED)

add_library(dock_ground_core STATIC
    src/config.cpp
    src/command_sink.cpp
    src/flight_logger.cpp
    src/gs_link.cpp
    src/json.cpp
    src/pose_source.cpp
    src/position_controller.cpp
    src/runtime.cpp
    src/web_protocol.cpp
    src/web_server.cpp
)

target_include_directories(dock_ground_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(dock_ground_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
)

target_link_libraries(dock_ground_core PUBLIC Threads::Threads)

if(DOCK_GROUND_WITH_VRPN)
    include(FetchContent)

    set(VRPN_BUILD_CLIENT_LIBRARY ON CACHE BOOL "" FORCE)
    set(VRPN_BUILD_SERVER_LIBRARY OFF CACHE BOOL "" FORCE)
    set(VRPN_BUILD_CLIENTS OFF CACHE BOOL "" FORCE)
    set(VRPN_BUILD_SERVERS OFF CACHE BOOL "" FORCE)
    set(VRPN_INSTALL OFF CACHE BOOL "" FORCE)
    set(VRPN_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
    set(VRPN_BUILD_JAVA OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    set(DOCK_GROUND_VRPN_SOURCE_DIR "" CACHE PATH
        "Optional local VRPN source tree; empty fetches the pinned release")
    FetchContent_Declare(vrpn
        GIT_REPOSITORY https://github.com/vrpn/vrpn.git
        GIT_TAG c672040788ad9c36cf43d98779e050c99bc896e6
        GIT_SHALLOW TRUE
    )
    if(DOCK_GROUND_VRPN_SOURCE_DIR)
        set(FETCHCONTENT_SOURCE_DIR_VRPN
            "${DOCK_GROUND_VRPN_SOURCE_DIR}"
            CACHE PATH "" FORCE)
    endif()
    FetchContent_MakeAvailable(vrpn)

    target_sources(dock_ground_core PRIVATE src/vrpn_pose_source.cpp)
    target_include_directories(dock_ground_core PRIVATE ${VRPN_INCLUDE_DIRS})
    target_link_libraries(dock_ground_core PUBLIC ${VRPN_CLIENT_LIBRARY})
    target_compile_definitions(dock_ground_core PUBLIC DOCK_GROUND_HAS_VRPN=1)

    add_executable(dock_ground_vrpn_probe tools/vrpn_probe.cpp)
    target_include_directories(dock_ground_vrpn_probe PRIVATE ${VRPN_INCLUDE_DIRS})
    target_link_libraries(dock_ground_vrpn_probe PRIVATE ${VRPN_CLIENT_LIBRARY})
endif()

add_executable(dock_ground_station src/main.cpp)
target_link_libraries(dock_ground_station PRIVATE dock_ground_core)
target_compile_options(dock_ground_station PRIVATE -Wall -Wextra -Wpedantic)

if(DOCK_GROUND_BUILD_TESTS)
    enable_testing()

    add_executable(dock_ground_tests tests/test_main.cpp)
    target_link_libraries(dock_ground_tests PRIVATE dock_ground_core)
    target_compile_options(dock_ground_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(dock_ground_tests
        PRIVATE
            DOCK_GROUND_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    add_test(NAME dock_ground_tests COMMAND dock_ground_tests)
endif()
