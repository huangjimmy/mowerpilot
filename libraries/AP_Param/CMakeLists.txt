set(AP_Param_sources AP_Param.cpp )

set( sources
        ${AP_Param_sources}
        )

include_directories(../../libraries)

add_library(AP_Param STATIC ${AP_Param_sources})