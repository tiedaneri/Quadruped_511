/*
 * @Author: your name
 * @Date: 2021-09-30 05:02:09
 * @LastEditTime: 2021-09-30 05:25:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/gait_plan_test/gait_test.cpp
 */

#include "gait_plan.h"

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
    gait_plan gait;

    signal(SIGINT, signal_handler);//register signal and signal handle function

    gait.init();
    gait.set();

    
    while(!stop_cmd)
    {
        gait.run();
        for(int leg = 0; leg < 4; leg++)
        printf("leg%d: %d, ", leg, gait.stance_flag[leg]);
        printf("\n");
        usleep(100000); 
    }

    printf("[main] program ended\n");
    return 1;
}
