
set(AP_NavEKF2_sources AP_NavEKF2.cpp AP_NavEKF2_AirDataFusion.cpp AP_NavEKF2_Control.cpp AP_NavEKF2_core.cpp AP_NavEKF2_MagFusion.cpp
        AP_NavEKF2_Measurements.cpp AP_NavEKF2_OptFlowFusion.cpp AP_NavEKF2_Outputs.cpp AP_NavEKF2_PosVelFusion.cpp
        AP_NavEKF2_RngBcnFusion.cpp AP_NavEKF2_VehicleStatus.cpp AP_NavEKF_GyroBias.cpp)

include_directories(../../libraries)

set( sources
        ${AP_NavEKF2_sources}
        )



add_library(AP_NavEKF2 STATIC ${AP_NavEKF2_sources})