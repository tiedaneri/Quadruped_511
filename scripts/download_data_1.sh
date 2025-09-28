
###
 # @Author: sun
 # @Date: 2021-10-02 00:34:56
 # @LastEditTime: 2021-10-02 00:54:29
 # @LastEditors: Please set LastEditors
 # @Description: 该脚本文件用于从机器人下载数据
 # @FilePath: /robot_software/scripts/download_data.sh
### 

## 指南：
##      在PC端打开终端
##      然后运行指令: {./download_data.sh}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"  # 该文件的地址
cd ${DIR}/../               # 打开上一级目录（工程文件夹根目录）
rm -rf robot_running_data   # 删除文件夹"robot_running_data"和里面所有的文件
mkdir robot_running_data    # 新建文件夹"robot_running_data"
#将机器人的"根目录/robot_software/build/robot_data.txt" 文件下载到robot_running_data文件夹中
scp -r robot@10.0.0.34:~/robot_software/build/robot_data.txt robot_running_data/

