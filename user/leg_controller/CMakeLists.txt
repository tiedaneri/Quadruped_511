
project(joint_controller)

include_directories("./")
include_directories("./include")
include_directories("./convexMPC")
include_directories("./WBC_Ctrl")
include_directories("./WBC_Ctrl/ContactSet")
include_directories("./WBC_Ctrl/TaskSet")

include_directories("../../robot/include")
include_directories("../../third_party/include")
include_directories("../../third_party/include/lord_imu/Include")
include_directories("../../third_party/qpOASES/include")
include_directories("../../third_party/Goldfarb_Optimizer")
include_directories("/usr/local/include/eigen3")

add_executable(leg_controller main.cpp 
                                  leg_controller.cpp

                                  source/mode_controller.cpp
                                  source/safety_check.cpp
                                  source/trajectory_plan.cpp
                                  source/balance_stand_controller.cpp
                                  source/locomotion_controller.cpp
                                  source/gait_plan.cpp

                                  convexMPC/convexMPC_interface.cpp
                                  convexMPC/SolverMPC.cpp
                                  convexMPC/RobotState.cpp

                                  WBC_Ctrl/WBC_Ctrl.cpp
                                  WBC_Ctrl/LocomotionCtrl.cpp
                                  WBC_Ctrl/TaskSet/BodyOriTask.cpp
                                  WBC_Ctrl/TaskSet/BodyPosTask.cpp
                                  WBC_Ctrl/TaskSet/LinkPosTask.cpp
                                  WBC_Ctrl/ContactSet/SingleContact.cpp

              )

target_link_libraries(leg_controller robot third_party)
