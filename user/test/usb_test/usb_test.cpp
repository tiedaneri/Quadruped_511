/*
 * @Author: your name
 * @Date: 2025-09-18 10:41:23
 * @LastEditTime: 2025-11-11 14:58:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_WBC/user/test/usb_test/usb_test.cpp
 */

 #include "up_usbDevice.h"

 #include <stdio.h>
 #include <unistd.h> //usleep()
 #include <csignal> //signal process function
 
 uint8_t stop_cmd = 0; //0-running, 1-stop
 
 void signal_handler(int signum){ //capture the "ctrl+C" in terminal
     printf("[signal_handler] capture signal: %d\n", signum);
     stop_cmd = 1;
     
     printf("[main] program ended!\n");
 }
 
 int main(void){
     usb_device usb_dev;
     FILE* data_file;
 
     data_file = fopen("robot_data.txt", "w");
 
     signal(SIGINT, signal_handler);
 
     if(!usb_dev.init_usb()){
         printf("[main]: init USB error!\n");
         return 0;
     }
     printf("[main]: init USB Success!\n");
 
     while(!stop_cmd){
         if(usb_dev.read_and_parse_data()){
             for(int i = 0; i < sizeof(usb_dev.data.sdata)/sizeof(float); i++){
                 printf("%.3f ", usb_dev.data.sdata[i]);
                 fprintf(data_file, "%.3f ", usb_dev.data.sdata[i]);
             }printf("\n");fprintf(data_file, "\n");
         }else{
             fprintf(data_file, "[WARNNING] usb_device: data find failed.\n");
         }        
     }
 
     fclose(data_file);
 
     printf("[main] program ended!\n");
     
     return 0;
 }
 
 