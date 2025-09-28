# robot folder

include_directories("include")
include_directories("../third_party/include")
include_directories("../third_party/include/lord_imu/Include")

add_library(robot SHARED                    
                         source/up_usbDevice.cpp
                         source/up_serial.cpp
                         source/up_at9s.cpp
                         source/up_spi.cpp
                         source/joint_driver.cpp
                         source/robot_console.cpp
                         source/leg_driver.cpp
                         source/robot.cpp
                         source/body_state_estimator.cpp
                         source/bezier_interpolation.cpp
                         source/robot_model.cpp
                         source/fb_dynamic.cpp
                         source/vector_tools.cpp
                         source/contact_detection.cpp
            )
target_link_libraries(robot third_party)
