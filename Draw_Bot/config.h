/*
 * @Author: ylt
 * @Date: 2019-11-19 10:51:04
 * @LastEditors: ylt
 * @LastEditTime: 2019-11-19 16:42:24
 * @FilePath: \TROBOT_WIFI\trobot_config.h
 */

// 取消这里的注释以使用路由器
// #define EXTERNAL_WIFI
// #define WIFI_SSID "FAST_347C"
// #define WIFI_PASSWORD "405405405"
// #define WIFI_IP "192.168.1.333"  // Force ROBOT IP
// #define TELEMETRY "192.168.1.105" // 远端端口 2223

//本机端口为 2222
#define TELEMETRY "192.168.4.2" // 默认服务器端口 远端端口 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_SPEED 1000

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780    // Max recommended value: 860
#define MAX_STEERING_PRO 260    // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26 // Max recommended value: 32

// Default control terms for EVO 2
#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.35
#define KD_POSITION 0.25
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0 // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 275
#define ITERM_MAX_ERROR 30 // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0 // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500 // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!


#define ZERO_MOTOR_PERIOD 65535
#define MAX_ACCEL 1 // 电机加速度限制

#define MICROSTEPPING 16 // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

// AUX definitions
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

String MAC; // MAC address of Wifi module

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;      // To generate a medium loop 40Hz
uint8_t slow_loop_counter; // slow loop 2Hz

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_speed = MAX_TARGET_SPEED;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;
uint8_t mode; // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

int L_mask = 0;
int R_mask = 0;
int speed = 10;
uint8_t L_period = 255;
uint8_t R_period = 255;
uint8_t L_counter = 0;
uint8_t R_counter = 0;
uint8_t L_loop_counter; // To generate a medium loop 40Hz
uint8_t R_loop_counter; // To generate a medium loop 40Hz
uint16_t STOP_loop_counter;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2; // Actual speed of motors
int8_t dir_M1, dir_M2;      // Actual direction of steppers motors
int16_t actual_robot_speed; // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered; // Estimated robot speed

// OSC output variables
uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
float OSCxy2_x;
float OSCxy2_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[4];
uint8_t OSCmove_mode;
int16_t OSCmove_speed;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;

typedef int var; // 将javascript数据类型映射为整数
int scale = 15;  // 字母比例, 2.5x = 5cm x 10cm
float BatteryValue;
uint8_t sendBattery_counter;
uint8_t battery_count = 0;

// 设置舵机
int servo1Pin = 5;
int servo2Pin = 6;
int PEN_DOWN = 160; // 笔落下时的舵机角度
int PEN_UP = 90;    // 笔抬起时的舵机角度
// Servo penServo;

// 机器人结构参数设置
float wheel_dia = 51.75;  //   mm  增大该值将减少机器人行走距离
float wheel_base = 72.75; //   mm  增大该值将增大机器人旋转角度

//设置按键、LED、电压检测IO
byte button_pin = A6; //D4复用为A6
byte LED1_pin = 9;
byte LED2_pin = 7;
byte battery_pin = 10;

// 步进电机参数设置
int steps_rev = 512;                     // 512 --- 64/1减速比, 128 --- 16/1减速比
int delay_time = 2;                      // 两步之间的间隔时间 ms 越小速度越快 最小为2
int L_stepper_pins[] = {15, 16, 14, 8};  //PB1, PB2, PB3, PB4
int R_stepper_pins[] = {A3, A2, A1, A0}; //PF4, PF5, PF6, PF7

// 步进电机的驱动方式是直接用引脚IO两两接通 按顺序通电 即可
// 左右电机安装方向不同 在前进的时候 左电机逆时针通电 右电机顺时针通电
// port PB  左步进电机线序 PB1 PB4 -> PB4 PB3 ->PB3 PB2 -> PB2 PB1
byte L_seq[4] = {B00010010, B00011000, B00001100, B00000110};
// port PF  右步进电机线序 PF7 PF6 -> PF6 PF5 ->PF5 PF4 -> PF4 PF7
byte R_seq[4] = {B11000000, B01100000, B00110000, B10010000};

// 使用数组减少初始化代码
byte inputs[] = {button_pin, battery_pin};
byte outputs[] = {servo1Pin, servo2Pin, LED1_pin, LED2_pin};
