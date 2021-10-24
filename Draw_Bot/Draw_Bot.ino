
/*
 * Filename: /media/ylt/本地磁盘/TROBOT/TROBOT_WIFI/TROBOT_WIFI.ino
 * Path: /media/ylt/本地磁盘/TROBOT/TROBOT_WIFI
 * Created Date: Friday, November 22nd 2019, 7:48:38 pm
 * Author: ylt
 * 
 * Copyright (c) 2019 EH_ROBOT, Inc
 */

//乌龟机器人
//板子类型选择 arduino/genuino micro

#include "config.h"

//#define DEBUG 2 // 0:不打印调试信息

void setup()
{

  IO_Init(); // 初始化所有IO口

  Serial.begin(9600); // 初始化串口
  Serial1.begin(115200);

  OSC_init(); //OSC协议初始化

  Serial.println("YLT - TROBOT");

  WIFI_init(); //创建WIFI网络

  //penServo.attach(servo1Pin); //配置舵机引脚

  Serial.println("Stepers init");
  //motoInitCheck(); //电机自检

  // CS12 CS11 CS10 描述 ATMEL32u4 定时器相关寄存器定义
  //  0    0    0   停止
  //  0    0    1   1 不分频
  //  0    1    0   8倍分频
  //  0    1    1   64倍分频
  //  1    0    0   256倍分频
  //  1    0    1   1024倍分频

  //比如预分频是1024 定时器的频率为 15625 HZ
  //则定时器tick一次为 1/(16M/1024) = 1/15625 =  0.000064 s / per cycle
  //要想定时时间为1s 则 计数个数为 1/0.000064 = 15624 次

  //预分频是64 定时器的频率为 250,000 HZ
  //则定时器tick一次为 1/(16M/64) = 1/250000 =  0.000004 s / per cycle
  //要想定时时间为1s 则 计数个数为 1/0.000004 = 250000 次
  //要想定时时间为1ms 则 计数个数为 0.001/0.000004 = 250 次
  //要想定时时间为0.1ms 则 计数个数为 0.0001/0.000004 = 25 次

  //预分频是8 定时器的频率为 2MHZ = 2,000,000 HZ
  //则定时器tick一次为 1/(16M/64) = 1/250000 =  0.000004 s / per cycle
  //要想定时时间为1s 则 计数个数为 1*2,000,000 = 2,000,000 次
  //要想定时时间为1ms 则 计数个数为 0.001*2,000,000 = 2000 次
  //要想定时时间为0.1ms 则 计数个数为 0.0001*2,000,000 = 200 次

  // MOTOR1 => TIMER1
  TCCR1A = 0;                          // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // 预分频数
  OCR1A = 10 * 200 - 1;                // 最大计数个数 第一个数为周期 单位为0.1ms
  dir_M1 = 1;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                          // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // 预分频数
  OCR3A = 10 * 200 - 1;                // 最大计数个数 第一个数为周期 单位为0.1ms
  dir_M2 = 1;
  TCNT3 = 0;
  delay(200);

  // 使能定时器中断
  TIMSK1 |= (1 << OCIE1A); // 使能 定时器1 中断
  TIMSK3 |= (1 << OCIE1A); // 使能 定时器3 中断

  Serial.println("Start...");
}

uint16_t stop_counter;
uint16_t mainloop_medium_counter; // 创建一个循环控制标志 控制循环频率
uint16_t mainloop_slow_counter;   // 创建一个循环控制标志 控制循环频率
//main loop
void loop()
{
  mainloop_medium_counter++;
  mainloop_slow_counter++;

  OSC_MsgRead(); // 读取 UDP OSC 消息
  if (OSCnewMessage)
  {
    OSCnewMessage = 0;
    if (OSCpage == 1) //OSC消息帧中第一页
    {
      if (modifing_control_parameters)
      {
        OSCfader[0] = 0.5; // 中位值
        OSCfader[1] = 0.5; // 中位值
        OSCtoggle[0] = 0;  // 普通模式
        mode = 0;
        modifing_control_parameters = false;
      }
      if (OSCmove_mode) //精确移动模式
      {
        positionControlMode = true;
        OSCmove_mode = false;
        target_steps1 = steps1 + OSCmove_steps1;
        target_steps2 = steps2 + OSCmove_steps2;
      }
      else
      {
        positionControlMode = false;
        throttle = (OSCfader[0] - 0.5) * max_throttle;
        steering = OSCfader[1] - 0.5;
        if (steering > 0)
          steering = (steering * steering + 0.5 * steering) * max_steering;
        else
          steering = (-steering * steering + 0.5 * steering) * max_steering;
      }

      if ((mode == 0) && (OSCtoggle[0])) // 变为 PRO 模式,修改成激进的PID
      {
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        //max_target_angle = MAX_TARGET_ANGLE_PRO;
        mode = 1;
      }
      if ((mode == 1) && (OSCtoggle[0] == 0)) // 变为普通模式,修改成稳定的PID
      {
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        //max_target_angle = MAX_TARGET_ANGLE;
        mode = 0;
      }
    }
    else if (OSCpage == 2) // OSC page 2
    {
      // 读取PID参数
      //readControlParameters();
    }
#if DEBUG == 1
    Serial.print(throttle);
    Serial.print(" ");
    Serial.println(steering);
#endif
  } // OSC message




  // 计算机器人速度
  actual_robot_speed = (speed_M1 + speed_M2) / 2; // 默认向前
  int16_t estimated_speed = actual_robot_speed * 1;
  estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; //低通滤波

  if (positionControlMode) //如果是位置控制模式,则使用不同的参数
  {
    // 位置控制. 输入: 目标脉冲. 输出: 电机速度
    motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
    motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

    // 将速度从位置控制输出转换成油门和转向速度
    throttle = (motor1_control + motor2_control) / 2;
    throttle = constrain(throttle, -275, 275);
    steering = motor2_control - motor1_control;
    steering = constrain(steering, -70, 70);
  }

#if DEBUG == 2
  // Serial.print(speed_M1);
  // Serial.print(" ");
  // Serial.print(motor1_control);
  // Serial.print(" ");
  Serial.print(steps1);
  Serial.print(" ");
  Serial.print(target_steps1);
  Serial.print(" ");

  Serial.print(steps2);
  Serial.print(" ");
  Serial.print(target_steps2);
  Serial.print(" ");

  // Serial.print(OSCmove_steps1);
  // Serial.println(" ");
#endif

  // 速度分解 , 得到每个电机的速度
  motor1 = throttle - steering;
  motor2 = throttle + steering;

  // 限制最大速度 (控制输出)
  motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

  //将速度信息输入给电机控制函数
  setMotorSpeedM1(motor1);
  setMotorSpeedM2(motor2);

#if DEBUG == 3
  Serial.print(motor1);
  Serial.print(" ");
  Serial.println(motor2);
  Serial.print(" ");
#endif

  //中速循环 大概0.5HZ
  if (mainloop_medium_counter >= 40000)
  {
    mainloop_medium_counter = 0;

    if (dir_M1 == 0 && dir_M2 == 0) //左轮右轮都停止
      stop_counter++;
    else
      stop_counter = 0; //如果没有连续累加到10就清零从新开始计数

    if (stop_counter >= 10) //当完全停止的时候累加十次 至少需要20s
    {
      stop_counter = 0;
      PORTF = PORTF & B00001111; //端口置为零 防止电机持续通电发烫
      PORTB = PORTB & B11100001;
    }
  }

  //慢速循环 大概1HZ
  if (mainloop_slow_counter >= 10000)
  {
    mainloop_slow_counter = 0;

    BatteryValue = readBattery(); //读取电池信息

    // char auxS[25];
    // sprintf(auxS, "$tB,%04d", 100);
    // Serial1.println(auxS);
    // Serial.print(abs(target_steps1 - steps1));
    // Serial.print(" ");
    // Serial.print(abs(target_steps2 - steps2));
    // Serial.println(" ");

    if (abs(target_steps1 - steps1) <= 2 && abs(target_steps2 - steps2) <= 2)
      Serial1.println(1001);
  }
}

void spiral(int size, int angle) // 向内螺旋测试
{
  while (size > 0)
  {
    moveForward(size);
    turnRight(angle);
    size = size - 2;
  }
}

void loopTest() // 绘制校准框4次
{
  for (int x = 0; x < 12; x++)
  {
    moveForward(100);
    turnLeft(90);
  }
  while (1)
  {
  }
}