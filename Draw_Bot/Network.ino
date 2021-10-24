/*
 * @Author: ylt
 * @Date: 2019-10-11 12:44:21
 * @LastEditors  : ylt
 * @LastEditTime : 2019-12-27 23:01:23
 * @FilePath: \BROBOT_EVO2\Network.ino
 */
//#include "trobot_config.h"
// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
// void readControlParameters()
// {
//   // Parameters Mode (page2 controls)
//   // Parameter initialization (first time we enter page2)
//   if (!modifing_control_parameters)
//   {
//     for (uint8_t i = 0; i < 4; i++)
//       OSCfader[i] = 0.5;
//     OSCtoggle[0] = 0;

//     modifing_control_parameters = true;
//     Serial1.println("$P2");
//   }
//   // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder1,2,3,4)
//   // Now we need to adjust all the parameters all the times because we dont know what parameter has been moved
//   Kp_user = KP * 2 * OSCfader[0];
//   Kd_user = KD * 2 * OSCfader[1];
//   Kp_thr_user = KP_THROTTLE * 2 * OSCfader[2];
//   Ki_thr_user = KI_THROTTLE * 2 * OSCfader[3];
//   // Send a special telemetry message with the new parameters
//   char auxS[50];
//   sprintf(auxS, "$tP,%d,%d,%d,%d", int(Kp_user * 1000), int(Kd_user * 1000), int(Kp_thr_user * 1000), int(Ki_thr_user * 1000));
//   Serial1.println(auxS);

// #if DEBUG>0
//   Serial.print("Par: ");
//   Serial.print(Kp_user);
//   Serial.print(" ");
//   Serial.print(Kd_user);
//   Serial.print(" ");
//   Serial.print(Kp_thr_user);
//   Serial.print(" ");
//   Serial.println(Ki_thr_user);
// #endif

//   // Calibration mode??
//   if (OSCpush[2]==1)
//   {
//     Serial.print("Calibration MODE ");
//     angle_offset = angle_adjusted_filtered;
//     Serial.println(angle_offset);
//   }

//   // Kill robot => Sleep
//   while (OSCtoggle[0] == 1)
//   {
//     //Reset external parameters
//     PID_errorSum = 0;
//     timer_old = millis();
//     setMotorSpeedM1(0);
//     setMotorSpeedM2(0);
//     digitalWrite(4, HIGH);  // Disable motors
//     OSC_MsgRead();
//   }
// }
void WIFI_init(void)
{
  // ESP8266 需要进行一些初始化操作
  Serial.println("WIFI init");
  Serial1.flush();
  Serial1.print("+++"); // 确保退出了透传模式
  delay(100);
  ESPsendCommand("AT", "OK", 1);
  ESPsendCommand("AT+RST", "OK", 2); // ESP Wifi 模块重启
  ESPwait("ready", 6);
  ESPsendCommand("AT+GMR", "OK", 5);

#ifdef EXTERNAL_WIFI //是否使用路由器连接
  //ESPsendCommand("AT+CWQAP", "OK", 3);
  ESPsendCommand("AT+CWMODE=1", "OK", 3);
  //String auxCommand = (String)"AT+CWJAP="+WIFI_SSID+","+WIFI_PASSWORD;
  char auxCommand[90] = "AT+CWJAP_CUR=\"";
  strcat(auxCommand, WIFI_SSID);
  strcat(auxCommand, "\",\"");
  strcat(auxCommand, WIFI_PASSWORD);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 14);
#ifdef WIFI_IP
  strcpy(auxCommand, "AT+CIPSTA_CUR=\"");
  strcat(auxCommand, WIFI_IP);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 4);
#endif
  ESPsendCommand("AT+CIPSTA?", "OK", 4);
  ESPsendCommand("AT+CIFSR", "OK", 3);
#else // 默认 : 创建一个WIFI 热点
  Serial1.println("AT+CIPSTAMAC?");
  ESPgetMac();
  delay(200);
  //ESPsendCommand("AT+CWQAP", "OK", 3);    //确保断开了之前的连接
  ESPsendCommand("AT+CWMODE=2", "OK", 3); // Soft AP 模式
  // 创建一个热点. SSID=TROBOTS__XX, PASS=87654321
  char *cmd = "AT+CWSAP=\"TROBOT$_$XX\",\"87654321\",5,3";
  // 把XX替换成MAC地址用于区分不同的机器人
  cmd[19] = MAC[10];
  cmd[20] = MAC[11];
  
  ESPsendCommand(cmd, "OK", 6);

  ESPsendCommand("AT+CIPMUX=0", "OK", 3);  // 单连接模式
  ESPsendCommand("AT+CIPMODE=1", "OK", 3); // 透传模式
#endif
  // 启动 UDP 服务器 本机端口: 2222, 远端端口：2223
  Serial.println("Start UDP server");
  char Telemetry[80];
  strcpy(Telemetry, "AT+CIPSTART=\"UDP\",\"");
  strcat(Telemetry, TELEMETRY);
  strcat(Telemetry, "\",2223,2222,0");
  ESPsendCommand(Telemetry, "OK", 3);

// 开启传输 (透传模式)
#ifdef EXTERNAL_WIFI
  ESPsendCommand("AT+CIPSEND=7", ">", 2); // Start transmission (transparent mode)
#else
  ESPsendCommand("AT+CIPSEND", ">", 2);    // Start transmission (transparent mode) 发送数据
#endif
}

int ESPwait(String stopstr, int timeout_secs)
{
  String response;
  bool found = false;
  char c;
  long timer_init;
  long timer;

  timer_init = millis();
  while (!found)
  {
    timer = millis();
    if (((timer - timer_init) / 1000) > timeout_secs)
    { // Timeout?
      Serial.println("!Timeout!");
      return 0; // timeout
    }
    if (Serial1.available())
    {
      c = Serial1.read();
      Serial.print(c);
      response += c;
      if (response.endsWith(stopstr))
      {
        found = true;
        delay(10);
        Serial1.flush();
        Serial.println();
      }
    } // end Serial1_available()
  }   // end while (!found)
  return 1;
}

// getMacAddress from ESP wifi module
int ESPgetMac()
{
  char c1, c2;
  bool timeout = false;
  long timer_init;
  long timer;
  uint8_t state = 0;
  uint8_t index = 0;

  MAC = "";
  timer_init = millis();
  while (!timeout)
  {
    timer = millis();
    if (((timer - timer_init) / 1000) > 5) // Timeout?
      timeout = true;
    if (Serial1.available())
    {
      c2 = c1;
      c1 = Serial1.read();
      Serial.print(c1);
      switch (state)
      {
      case 0:
        if (c1 == ':')
          state = 1;
        break;
      case 1:
        if (c1 == '\r')
        {
          MAC.toUpperCase();
          state = 2;
        }
        else
        {
          if ((c1 != '"') && (c1 != ':'))
            MAC += c1; // Uppercase
        }
        break;
      case 2:
        if ((c2 == 'O') && (c1 == 'K'))
        {
          Serial.println();
          Serial1.flush();
          return 1; // Ok
        }
        break;
      } // end switch
    }   // Serial_available
  }     // while (!timeout)
  Serial.println("!Timeout!");
  Serial1.flush();
  return -1; // timeout
}

int ESPsendCommand(char *command, String stopstr, int timeout_secs)
{
  Serial1.println(command);
  ESPwait(stopstr, timeout_secs);
  delay(250);
}

