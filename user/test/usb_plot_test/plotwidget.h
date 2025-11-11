/*
 * @Author: your name
 * @Date: 2025-11-06 17:36:48
 * @LastEditTime: 2025-11-06 21:31:44
 * @LastEditors: siaohan
 * @Description: In User Settings Edit
 * @FilePath: /Quadruped_511_algorithm2/user/test/usb_test/plotwidget.h
 */
#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>
#include <QTimer>
#include "qcustomplot.h"
#include "up_usbDevice.h"

class PlotWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit PlotWidget(usb_device* _usb_dev, QWidget *parent = nullptr);
    void addData(int graphIndex, double x, double y);
    void clearData();

  public slots:
    void updatePlot();

  private:
    usb_device* usb_dev;
    QCustomPlot *customPlot;
    QTimer *dataTimer;
    double currentX;
};

#endif // PLOTWIDGET_H
