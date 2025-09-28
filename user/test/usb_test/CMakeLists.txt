project(usb_test)

include_directories("../../../robot/include")

add_executable(usb_test usb_test.cpp)
target_link_libraries(usb_test robot)