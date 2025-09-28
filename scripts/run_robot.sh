
###
 # @Author: sun
 # @Date: 2021-03-18 23:49:56
 # @LastEditTime: 2024-03-03 00:00:55
 # @LastEditors: Please set LastEditors
 # @Description: 配置upboard的环境以运行程序
 # @FilePath: /robot_software/scripts/run_robot.sh
### 

## 指南：
##      在up board"~/"目录下的终端中执行指令：{cd robot_software/build}
##      然后运行执行运行程序的指令：{./run_robot.sh ./[user program name].sun}

# 添加动态库路径，包括"/lib"、"/usr/lib"和文件"/etc/ld.so.conf"中的路径
sudo LD_LIBRARY_PATH=. ldconfig
# 添加目标程序路径为动态库路径
# "$1"是用户在终端中输入的目标的路径
# 即"./[user program name].sun"的路径
sudo LD_LIBRARY_PATH=. $1


