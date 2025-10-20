# third-party folder

add_subdirectory(qpOASES)
add_subdirectory(Goldfarb_Optimizer)

include_directories("include")
include_directories("include/lord_imu/Include")

add_definitions(-Dlinux)

add_library(third_party SHARED 
                                source/Utilities/PeriodicTask.cpp
                                source/Utilities/Utilities_print.cpp
                                
                                source/lord_imu/Source/mip.c
                                source/lord_imu/Source/mip_sdk_3dm.c
                                source/lord_imu/Source/mip_sdk_ahrs.c
                                source/lord_imu/Source/mip_sdk_base.c
                                source/lord_imu/Source/mip_sdk_filter.c
                                source/lord_imu/Source/mip_sdk_gps.c
                                source/lord_imu/Source/mip_sdk_inteface.c
                                source/lord_imu/Source/mip_sdk_system.c
                                source/lord_imu/Source/ring_buffer.c
                                source/lord_imu/Source/mip_sdk_user_functions.c
                                source/lord_imu/Source/byteswap_utilities.c
                                source/lord_imu/LordImu.cpp)

target_link_libraries(third_party pthread qpOASES Goldfarb_Optimizer)
