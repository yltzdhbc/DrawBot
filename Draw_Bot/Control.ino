/*
 * @Author: ylt
 * @Date: 2019-11-18 17:40:51
 * @LastEditors: ylt
 * @LastEditTime: 2019-11-23 16:17:13
 * @FilePath: \TROBOT_WIFI\Control.ino
 */

//#define CONTROLDEBUG

uint16_t temp = 375;
uint8_t dec_temp = 10;
uint8_t max_timer_period = 19;
uint8_t min_timer_period = 9;
// 设置电机1的速度
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // 限制步进电机的最大加速度
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING == 16
  speed = speed_M1 * 1; // 步进因子 steps/second
#else
  speed = speed_M1 * 1; //
#endif

  if (speed == 0)
  {
    timer_period = 1000;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = (temp - speed) / dec_temp;
    dir_M1 = 1;
  }
  else
  {
    timer_period = (temp - (-speed)) / dec_temp;
    dir_M1 = -1;
  }

#ifdef CONTROLDEBUG
  Serial.print("M1 : ");
  Serial.print(timer_period);
#endif

  // 限制最大周期,周期越大电机越慢,防止步进电机抖动
  // 限制最小周期,周期越小电机越快,防止步进电机丢步
  if (timer_period > max_timer_period)
    timer_period = max_timer_period;
  else if (timer_period < min_timer_period)
    timer_period = min_timer_period;

  OCR1A = timer_period * 200 - 1;

  // 检测是否需要重置定时器
  // if (TCNT1 > OCR1A)
  //   TCNT1 = 0;
}

// 设置电机2的速度
// 可以使用tspeed来算也可以不使用
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // 限制步进电机的最大加速度
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING == 16
  speed = speed_M2 * 1; // 步进因子 steps/second
#else
  speed = speed_M2 * 1; // 1/8
#endif

  if (speed == 0)
  {
    timer_period = 1000;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = (temp - speed) / dec_temp;
    dir_M2 = 1;
  }
  else
  {
    timer_period = (temp - (-speed)) / dec_temp;
    dir_M2 = -1;
  }
#ifdef CONTROLDEBUG
  Serial.print("M2 : ");
  Serial.println(timer_period);
#endif

  // 限制最大周期,周期越大电机越慢,防止步进电机抖动
  // 限制最小周期,周期越小电机越快,防止步进电机丢步
  if (timer_period > max_timer_period)
    timer_period = max_timer_period;
  else if (timer_period < min_timer_period)
    timer_period = min_timer_period;

  OCR3A = timer_period * 200 - 1;
}

// 定时器 1 : 步进电机1控制循环
ISR(TIMER1_COMPA_vect)
{
  //步进电机一步开始
  if (L_counter == 1) //高电平周期
  {
    if (dir_M1 == 1)
      PORTB = PORTB | L_seq[3 - L_mask];
    else if (dir_M1 == -1)
      PORTB = PORTB | L_seq[L_mask];
    L_counter = 0;
  }
  else //低电平周期
  {
    if (dir_M1 == 1)
    {
      PORTB = PORTB & ~L_seq[3 - L_mask];
      steps1++;
    }
    else if (dir_M1 == -1)
    {
      PORTB = PORTB & ~L_seq[L_mask];
      steps1--;
    }

    if (dir_M1 == 0)
      return;

    L_counter = 1;
    L_mask++;
  }
  //步进电机一步结束
  if (L_mask == 4)
    L_mask = 0;
}

// 定时器 3 : 步进电机2控制循环
ISR(TIMER3_COMPA_vect)
{
  //步进电机一步开始
  if (R_counter == 1) //高电平周期
  {
    if (dir_M2 == 1)
      PORTF = PORTF | R_seq[3 - R_mask];
    else if (dir_M2 == -1)
      PORTF = PORTF | R_seq[R_mask];
    R_counter = 0;
  }
  else //低电平周期
  {
    if (dir_M2 == 1)
    {
      PORTF = PORTF & ~R_seq[3 - R_mask];
      steps2++;
    }
    else if (dir_M2 == -1)
    {
      PORTF = PORTF & ~R_seq[R_mask];
      steps2--;
    }
    if (dir_M2 == 0)
      return;

    R_counter = 1;
    R_mask++;
  }
  //步进电机一步结束
  if (R_mask == 4)
    R_mask = 0;
}

float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -150, 150); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}