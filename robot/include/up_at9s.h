/*
 * @Author: sun
 * @Date: 2020-12-14 06:21:02
 * @LastEditTime: 2021-03-22 06:15:05
 * @LastEditors: Please set LastEditors
 * @Description: 读取遥控器（at9s）的数据，并解析成指令
 * @FilePath: /robot_software/robot/include/up_at9s.h
 */
/*
指南:
      0. 创建对象时自动运行: up_at9s();
      1. 初始化串口: init_serial();
      2. 接收数据并解析成指令: update_cmd();
      4. 指令被保存在变量中: cmd
*/
/*
channels 数据与指令对应关系:

channel_data[6]-SWA: UP-306,  DOW-1694
channel_data[9]-SWD: UP-306,  DOW-1694
channel_data[8]-SWC: UP-306,  MID-1000, DOW-1694
channel_data[4]-SWE: UP-1694, MID-1000, DOW-306
channel_data[5]-SWG: UP-1694, MID-1000, DOW-306

channel_data[0]-LRO: LIFT-307, MIDDLE-996,  RIGHT-1685  最小差值: 689
channel_data[3]-RRO: LIFT-334, MIDDLE-1006, RIGHT-1693  最小差值: 672
channel_data[1]-LCO: UP-287,   MIDDLE-1006, DOWN-1576   最小差值: 570
channel_data[2]-RCO: UP-392,   MIDDLE-999,  DOWN-1670   最小差值: 607

channel_data[7]-VB: ANTICLOCKWISE-306, MIDDLE-1074, CLOCKWISE-1694   最大差值: 1388
*/
#ifndef UP_AT9S_H
#define UP_AT9S_H

#include "up_serial.h"

typedef enum //3位钮子开关类型
{
  UP = 0, //上
  MID = 1,//中
  DOW = 2 //下

}switch3;

typedef struct //遥控器指令结构体类型
{
  switch3 SWA, SWB, SWD, SWF, SWH;  //2位钮子变量
  switch3 SWC, SWE, SWG;            //3位钮子变量
  float LRO;     //左摇杆横向，左 : +, 中: 0, 右: -, +0.5:0:-0.5
  float LCO;     //左摇杆竖向，上 : +, 中: 0, 下: -, +0.5:0:-0.5
  float RRO;     //右摇杆横向，左 : +, 中: 0, 右: -, +0.5:0:-0.5
  float RCO;     //右摇杆竖向，上 : +, 中: 0, 下: -, +0.5:0:-0.5
  float VA, VB;  //旋钮  ，逆时针： +, 顺时针: -, 0-1
  
}at9s_cmd;

/*
AT9S 遥控器类，接收遥控器数据并转换成指令
*/
class up_at9s: public up_serial
{
  public:
    up_at9s(void);
    
    uint8_t init_serial(void);
    uint8_t update_cmd(void);

    //变量：存储遥控器指令数据
    at9s_cmd cmd;

  private:
    uint8_t receive_original_data(void);
    void unpack_original_data(void);
    void analysis_cmd(void);

    //变量：存储原始sbus数据
    uint8_t original_data[25];
    //变量：存储解码后的channel数据
    uint16_t channel_data[18];
    
};

#endif // #ifndef UP_AT9S_H
