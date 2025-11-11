project(usb_plot_test)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
# 启用 Qt MOC 自动处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 包含当前目录，让 MOC 能找到头文件
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("../../../robot/include")
include_directories(usb_plot_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(usb_plot_test usb_plot_test.cpp
                             plotwidget.cpp
                             qcustomplot.cpp)

target_link_libraries(usb_plot_test robot
                                    Qt5::Core
                                    Qt5::Widgets)