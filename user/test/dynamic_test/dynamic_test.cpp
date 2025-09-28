/*
 * @Author: sun
 * @Date: 2021-05-25 17:55:32
 * @LastEditTime: 2021-05-26 18:47:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /robot_software/user/test/dynamic_test/dynamic_test.cpp
 */

#include "fb_dynamic.h"
#include "robot_model.h"

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
    leg_state_data* state;
    body_state_estimator_data* body_data;
    robot_model model;
    fb_dynamic dynamic(&model, state, body_data);

    signal(SIGINT, signal_handler);//register signal and signal handle function
    
    model.build();
    printf("[main] build model done!!\n");
    dynamic.init();
    printf("[main] init dynamic done!!\n");
    // Vec6<float> ag;
    // ag << 0, 0, 0, 0, 0, -9.81;
    // dynamic.X[0] = Mat6<float>::Identity();
    // dynamic.avp[0] = - dynamic.X[0] * ag;
    dynamic.qdd.block(0, 0, 6, 1) << 0, 0, 0, 0, 0, 9.81;
    // while(!stop_cmd)
    // {
    // }
    dynamic.run_id();
    printf("T: ");
    for(int i = 0; i < 18; i++) printf("%f, ", dynamic.T(i));
    // for(int i = 0; i < 6; i++) printf("%f, ", dynamic.pc0(i));
    printf("\n");

    printf("[main] program ended\n");
    return 1;
}
