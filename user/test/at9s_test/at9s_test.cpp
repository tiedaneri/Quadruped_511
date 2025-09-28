/*
 * @Author: sun
 * @Date: 2020-12-14 06:19:27
 * @LastEditTime: 2021-01-09 05:40:54
 * @LastEditors: Please set LastEditors
 * @Description: test at9s
 * @FilePath: /robot_software/user/test/at9s_test/at9s_test.cpp
 */
#include "up_at9s.h"

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
    up_at9s at9s;

    signal(SIGINT, signal_handler);//register signal and signal handle function
    
    if(!at9s.init_serial()) 
    {
        printf("[main] init serial port error!!\n");
        return 0;
    }
    printf("[main] init serial port success!!\n"); 
    while(!stop_cmd)
    {
        if(at9s.update_cmd()) 
        {
            if(at9s.cmd.SWA == UP)  printf("SWA: UP , ");
            if(at9s.cmd.SWA == DOW) printf("SWA: DOW, ");
            if(at9s.cmd.SWD == UP)  printf("SWD: UP , ");
            if(at9s.cmd.SWD == DOW) printf("SWD: DOW, ");
            if(at9s.cmd.SWC == UP)  printf("SWC: UP , ");
            if(at9s.cmd.SWC == MID) printf("SWC: MID, ");
            if(at9s.cmd.SWC == DOW) printf("SWC: DOW, ");
            if(at9s.cmd.SWE == UP)  printf("SWE: UP , ");
            if(at9s.cmd.SWE == MID) printf("SWE: MID, ");
            if(at9s.cmd.SWE == DOW) printf("SWE: DOW, ");
            if(at9s.cmd.SWG == UP)  printf("SWG: UP , ");
            if(at9s.cmd.SWG == MID) printf("SWG: MID, ");
            if(at9s.cmd.SWG == DOW) printf("SWG: DOW, ");
            printf("LRO: %f, ", at9s.cmd.LRO);
            printf("RRO: %f, ", at9s.cmd.RRO);
            printf("LCO: %f, ", at9s.cmd.LCO);
            printf("RCO: %f, ", at9s.cmd.RCO);
            printf("VB:  %f\n", at9s.cmd.VB);
        }
        else printf("[ERROR] main: get sbus data error!!\n");
        usleep(2000);
    }
    printf("[main] program ended\n");
    return 1;
}
