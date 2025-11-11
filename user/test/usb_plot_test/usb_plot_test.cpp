/*
 * @Author: your name
 * @Date: 2025-09-18 10:41:23
 * @LastEditTime: 2025-11-08 11:09:15
 * @LastEditors: 
 * @Description: In User Settings Edit
 * @FilePath: /robot_software_WBC/user/test/usb_plot_test/usb_plot_test.cpp
 */

#include "up_usbDevice.h"

#include <stdio.h>
#include <QApplication>
#include <QMainWindow>
#include "plotwidget.h"
#include <unistd.h> //usleep()
#include <csignal> //signal process function

uint8_t stop_cmd = 0; //0-running, 1-stop

void signal_handler(int signum){ //capture the "ctrl+C" in terminal
    printf("[signal_handler] capture signal: %d\n", signum);
    stop_cmd = 1;

    printf("[main] program ended!\n");
}

int main(int argc, char *argv[])
{   
    usb_device usb_dev;

    QApplication app(argc, argv);    
    QMainWindow window;
    PlotWidget *plotWidget = new PlotWidget(&usb_dev);
    
    // signal(SIGINT, signal_handler);

    if(!usb_dev.init_usb()){
        printf("[main]: init USB error!\n");
        return 0;
    }
    printf("[main]: init USB Success!\n");

    // while(!stop_cmd){
    //     if(usb_dev.read_and_parse_data()){
    //         for(int i = 3; i < sizeof(usb_dev.data.sdata)/sizeof(float); i++){
    //             printf("%.3f ", usb_dev.data.sdata[i]);
    //         }printf("\n");
    //     }else{
    //     }        
    // }
    // printf("[main] program ended!\n");

    window.setCentralWidget(plotWidget);
    window.resize(800, 600);
    window.setWindowTitle("实时数据曲线 - QCustomPlot");
    window.show();    
    
    return app.exec();
}

