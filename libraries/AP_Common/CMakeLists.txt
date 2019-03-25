set(AP_Common_sources AP_Common.cpp Bitmask.cpp Location.cpp)

include_directories(../../libraries)

set( sources
        ${AP_Common_sources}
        )
add_library(AP_Common STATIC ${AP_Common_sources})