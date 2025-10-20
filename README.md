# 简介

    此工程为具有WBC的从机器人实机代码。

# 指南

```bash
0. 在根目录CMakeLists.txt选择机器人的宏定义
    可选1号机从机/遥控器模式，或者2号机遥控器模式。
1. 在PC端的"robot_software/build"文件夹中打开终端
2. 编译该工程。在终端中执行以下指令：
    {cmake -DCMAKE_BUILD_TYPE=Release ..} # 用于设置为Release模式进行编译
    {make}
3. 将程序下载到机器人中。在终端执行以下指令：
    {../scripts/loading1.sh} # 注意对应几号机器人
4. 通过SSH连接机器人。在终端执行以下指令(密码: 123456):
    {ssh user@10.0.0.34} # 1号机名称为robot，即：{ssh robot@10.0.0.34}(密码：123456).
5. 打开机器人中的程序文件夹. 执行以下指令：
    {cd robot_software/build}
6. 在机器人上运行程序. 执行以下指令：
    {./run_robot.sh ./[user program name]}
7. 结束程序指令按键：
    {Ctrl + C}
```

# TIP

    0. 软件需要在mini-cheetah的up-board环境中运行

    1. 仓库储存于win系统下，可通过VMware共享文件夹于Ubuntu下访问，但不可直接编译，需拷贝后编译（以下命令需匹配自己pc的路径）。

```bash
# 将项目复制到Ubuntu本地目录
cp -r /mnt/hgfs/Quadruped_511 /home/tiedan/Quadruped_511_compile
cd /home/tiedan/Quadruped_511_compile

# 创建构建目录
mkdir build && cd build

# 重新编译
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4

```

2. 文件介绍
   1）"third_party"   文件夹: 第三方的原文件和头文件
   2）"robot"               文件夹: 与机器人底层有关的代码
   3）"user"                 文件夹: main函数和上层控制器的代码，为程序的入口
   4）"user/test"       文件夹: 测试底层驱动的代码
   5） "scripts"            文件夹: 与下载程序和配置upboard有关的脚本文件
