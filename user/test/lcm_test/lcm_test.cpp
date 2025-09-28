/*
 * @Author: sun
 * @Date: 2021-02-16 08:20:13
 * @LastEditTime: 2021-02-17 07:00:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/lcm_test/lcm_test.cpp
 */

#include "exlcm/lcm_type.hpp"
#include <lcm/lcm-cpp.hpp>

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

int main()
{
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");
    if (!lcm.good())
    return 1;
    exlcm::lcm_type my_data;

    my_data.s_type = "hello word";
    my_data.b_type = true;
    my_data.u8 = 23;
    my_data.u16 = 123;
    my_data.u32 = 1234;
    my_data.u64 = 12345;
    my_data.f_type = 123.456;
    my_data.d_type = 456.789;

    signal(SIGINT, signal_handler);//register signal and signal handle function
    
    while(!stop_cmd)
    {
        my_data.b_type = !my_data.b_type;
        my_data.u8++;
        my_data.u16++;
        my_data.u32++;
        my_data.u64++;
        my_data.f_type++;
        my_data.d_type++;
        lcm.publish("lcm_test", &my_data);
        usleep(1000000);
    }
    printf("[main] program ended\n");
    return 1;
}
