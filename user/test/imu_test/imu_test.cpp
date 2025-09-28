/*
 * @Author: sun
 * @Date: 2020-12-28 03:38:15
 * @LastEditTime: 2024-02-29 18:07:23
 * @LastEditors: Please set LastEditors
 * @Description: test imu
 * @FilePath: /robot_software/user/test/imu_test/imu_test.cpp
 */

#include "lord_imu/LordImu.h"
#include "Math/orientation_tools.h"

#include <stdio.h>
#include <unistd.h>//"usleep" function
#include <csignal>//signal process function

uint8_t stop_cmd = 0;//0-running, 1-stop

void signal_handler(int signum)//input "ctr+C" in terminal
{
    printf("[signal_handler] capture signal: %d\n", signum);
    stop_cmd = 1;//stop
    // printf("program closed\n");
    // exit(signum);//kill this program
}

int main(void)
{
  u32 com_port, baudrate;
  com_port = 0;
  baudrate = 460800;
  LordImu imu;

  signal(SIGINT, signal_handler);//register signal and signal handle function
  
  if(imu.tryInit(com_port, baudrate)) 
  {
    while(!stop_cmd)
    {
      imu.run();
      Vec3<float> rpy = ori::quatToRPY(imu.quat); (void)rpy;
      Vec3<float> acc = imu.acc; (void)acc;
      Vec3<float> ang = imu.gyro; (void)ang;
      printf("%.3f %.3f %.3f ", rpy[0], rpy[1], rpy[2]);
      printf("%.3f %.3f %.3f ", acc[0], acc[1], acc[2]);
      printf("%.3f %.3f %.3f\n", ang[0], ang[1], ang[2]);
      usleep(10000); 
    }
  }
  printf("[main] program ended\n");
  return 1;
}
