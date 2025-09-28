/*
 * @Author: your name
 * @Date: 2021-01-02 23:21:20
 * @LastEditTime: 2021-01-04 06:55:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/task_test/task_test.cpp
 */

#include "Utilities/PeriodicTask.h"

#include <stdio.h>
#include <unistd.h>//"usleep" function
#include <csignal>//signal process function

uint8_t stop_cmd = 0;//0-running, 1-stop

void signal_handler(int signum)//input "ctr+C" in terminal
{
    printf("capture signal: %d\n", signum);
    stop_cmd = 1;//stop
    // printf("program closed\n");
    // exit(signum);//kill this program
}
void task1()
{
    printf("task1\n");
}
void task2()
{
    printf("task2\n");
}

int main()
{
    PeriodicTaskManager taskmanager;

    signal(SIGINT, signal_handler);//register signal and signal handle function
    
    PeriodicFunction _task1(&taskmanager, 1.f, "task1", &task1);
    PeriodicFunction _task2(&taskmanager, 1.f, "task2", &task2);
    _task1.start();
    _task2.start();

    while(!stop_cmd)
    {
        usleep(1000000);
    }
    taskmanager.stopAll();
    printf("program ended\n");
    return 1;
}
