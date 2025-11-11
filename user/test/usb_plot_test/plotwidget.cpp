/*
 * @Author: your name
 * @Date: 2025-11-06 17:36:48
 * @LastEditTime: 2025-11-06 21:48:30
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: /Quadruped_511_algorithm2/user/test/usb_test/plotwidget.cpp
 */
// src/plotwidget.cpp
#include "plotwidget.h"
#include <QVBoxLayout>
#include <QDateTime>
#include <cmath>
#include <stdlib.h>

PlotWidget::PlotWidget(usb_device* _usb_dev, QWidget *parent) 
  : QWidget(parent), currentX(0), usb_dev(_usb_dev)
{
    // 创建布局
    QVBoxLayout *layout = new QVBoxLayout(this);
    
    // 创建QCustomPlot
    customPlot = new QCustomPlot(this);
    layout->addWidget(customPlot);
    
    // 配置图表 - 添加三条曲线
    // 曲线 0: 蓝色
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue, 2));  // 线宽2
    customPlot->graph(0)->setName("曲线1");
    
    // 曲线 1: 红色
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::red, 2));
    customPlot->graph(1)->setName("曲线2");
    
    // 曲线 2: 绿色
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::green, 2));
    customPlot->graph(2)->setName("曲线3");
    // 可选：设置填充颜色
    // customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20)));

    // 可选：显示图例
    customPlot->legend->setVisible(true);
    customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);
    
    // 设置坐标轴
    customPlot->xAxis->setLabel("时间");
    customPlot->yAxis->setLabel("数值");
    customPlot->xAxis->setRange(0, 1000); //X轴range范围10s
    customPlot->yAxis->setRange(-30, 30);  // Y轴range范围
    
    // 启用抗锯齿
    customPlot->setAntialiasedElements(QCP::aeAll);
    
    // 创建定时器触发槽函数，用于更新数据
    dataTimer = new QTimer(this);
    connect(dataTimer, &QTimer::timeout, this, &PlotWidget::updatePlot);
    dataTimer->start(1); // 4ms、250Hz更新一次
}

/**
 * @brief 添加数据到指定曲线
 * @param graphIndex 曲线索引
 * @param x X轴数据
 * @param y Y轴数据
 */
void PlotWidget::addData(int graphIndex, double x, double y)
{
    // 检查图形索引是否有效
    if (graphIndex < 0 || graphIndex >= customPlot->graphCount()) {
        return;
    }

    customPlot->graph(graphIndex)->addData(x, y);
    
    // 滚动显示,当 x 值超过当前 X 轴最大值时,重新设置 X 轴范围,保持显示窗口宽度为 100
    if (x > customPlot->xAxis->range().upper) {
        customPlot->xAxis->setRange(x, 1000, Qt::AlignRight);
    }
    
    customPlot->replot(); //刷新图表
}

void PlotWidget::clearData()
{
    // 清除所有图形的数据
    for (int i = 0; i < customPlot->graphCount(); ++i) {
        customPlot->graph(i)->data()->clear();
    }
    currentX = 0;
    customPlot->replot();
}

void PlotWidget::updatePlot()
{
    if(usb_dev->read_and_parse_data()){
        for(int i = 3; i < sizeof(usb_dev->data.sdata)/sizeof(float); i++){
            printf("%.3f ", usb_dev->data.sdata[i]);
        }printf("\n");
    }
    // 传感器赋值
    double y1 = (double)usb_dev->data.sdata[3];
    double y2 = (double)usb_dev->data.sdata[4];
    double y3 = (double)usb_dev->data.sdata[5];

    addData(0, currentX, y1);
    addData(1, currentX, y2);
    addData(2, currentX, y3);
    
    // 时间递增 (单位:10毫秒)
    currentX += 0.4;  // 每次增加4ms
}
