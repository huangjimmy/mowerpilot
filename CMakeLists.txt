cmake_minimum_required( VERSION 3.5 )
project (EKF2_Main)

set (EKF2_Main_VERSION_MAJOR 1)
set (EKF2_Main_VERSION_MINOR 0)

if (UNIX)
    if (APPLE)
        execute_process(COMMAND xcrun --show-sdk-path
                OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
                OUTPUT_STRIP_TRAILING_WHITESPACE)
        list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/llvm@4")
        list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/gettext")
    endif()
endif()

set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_EXTENSIONS ON )
set( CXX_STANDARD_REQUIRED ON)

add_subdirectory( libraries )

include_directories(libraries)

add_executable(Main main.cpp)

target_link_libraries(Main AP_Common AP_Param AP_NavEKF2 AP_Math)
