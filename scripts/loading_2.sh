
###
 # @Author: sun
 # @Date: 2021-03-18 23:49:56
 # @LastEditTime: 2024-04-04 01:25:56
 # @LastEditors: Please set LastEditors
 # @Description: 该脚本文件用于下载程序到机器人
 # @FilePath: /robot_software/scripts/loading.sh
### 

## 指南：
##      在PC端工程目录的"robot_software/build" 文件夹中打开终端
##      然后运行指令: {../scripts/loading.sh}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"  # 该文件的地址
cd ${DIR}/../build/         # 打开文件夹"build"
rm -rf robot_software       # 删除文件夹"robot_software"和里面所有的文件
mkdir  robot_software       # 新建文件夹"robot_software"
mkdir  robot_software/build  

## 搜索法复制动态库
# find . -name \*.so* -exec cp {} ./robot_software/build \;  # 查找当前目录下所有名称中带有".so"的文件
#                                                            # 然后复制到"./robot_software/build"
## 搜索法复制程序
# find . -name \*.sun* -exec cp {} ./robot_software/build \;

## 挨个复制动态库和程序
cp ./robot/librobot.so                          ./robot_software/build
cp ./third_party/libthird_party.so              ./robot_software/build
cp ./third_party/qpOASES/libs/libqpOASES.so     ./robot_software/build
cp ./third_party/qpOASES/libs/libqpOASES.so.3.2     ./robot_software/build
cp ./third_party/Goldfarb_Optimizer/libGoldfarb_Optimizer.so    ./robot_software/build #WBC用的QuadProg++求解器
## 挨个复制程序
cp ./user/leg_controller/leg_controller      ./robot_software/build
# cp ./user/test/at9s_test/at9s_test           ./robot_software/build
# cp ./user/test/dynamic_test/dynamic_test     ./robot_software/build
# cp ./user/test/file_test/file_test           ./robot_software/build
# cp ./user/test/gait_plan_test/gait_plan_test ./robot_software/build
# cp ./user/test/imu_test/imu_test             ./robot_software/build
# cp ./user/test/lcm_test/lcm_test             ./robot_software/build
# cp ./user/test/mutex_test/mutex_test         ./robot_software/build
# cp ./user/test/task_test/task_test           ./robot_software/build

## 复制机器人运行环境配置脚本
cp ../scripts/run_robot.sh ./robot_software/build

## 无关紧要的文件
# cp -r ../robot robot_software
# cp -r ../user  robot_software
# cp -r ../scripts robot_software
# cp ../CMakeLists.txt ./robot_software
# cp ../readme.txt ./robot_software

## 复制文件夹"robot_software"和其中的文件到upboard的"~/"目录下
scp -r robot_software user@10.0.0.34:~/


