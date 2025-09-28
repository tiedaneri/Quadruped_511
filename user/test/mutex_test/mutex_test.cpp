/*
 * @Author: your name
 * @Date: 2021-03-13 22:22:34
 * @LastEditTime: 2021-03-13 23:33:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/mutex_test/mutex_test.cpp
 */

#include <stdio.h>
#include <unistd.h>//"usleep" function
#include <csignal>//signal process function
#include <thread>//多线程
#include <mutex>//互斥线程锁

uint8_t stop_cmd = 0;//0-running, 1-stop
int data[20]={0};

std::mutex m;
void r()
{
    while(!stop_cmd)
    {
        printf("r: ");
        m.lock();
        for(int i = 0; i < 20; i++)printf("%d,", data[i]);
        m.unlock();
        printf("\r\n");
        usleep(200000);
    }
    return;
}
void w()
{
    while(!stop_cmd)
    {
        static int t = 0;
        m.lock();
        for(int i = 0; i < 20; i++) 
        {
            data[i] = t;
            usleep(10000); 
        }
        m.unlock();
        t++;
        if(t >= 0xFFFFFFFF) t = 0;

        usleep(100000);
    }
    return;
}

void signal_handler(int signum)//input "ctr+C" in terminal
{
    printf("[signal_handler] capture signal: %d\n", signum);
    stop_cmd = 1;//stop
    // printf("program closed\n");
    // exit(signum);//kill this program
}

int main()
{
    std::thread thread_write(r);
    std::thread thread_read(w);

    signal(SIGINT, signal_handler);//register signal and signal handle function

    thread_write.join();
    thread_read.join();
    
    while(!stop_cmd)
    {
        usleep(1000000);
    }
    printf("[main] program ended\n");
    return 1;
}
