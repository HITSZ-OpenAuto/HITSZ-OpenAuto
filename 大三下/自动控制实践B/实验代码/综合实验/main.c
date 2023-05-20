/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body\ 
  * @author         : 黄继凡 huangjifan
  * @date           : 2023-05-21
  * @description:   
  * 自动控制实践综合实验代码综合
  * Integrated Code for Automatic Control Practical Comprehensive Experiment
  * @version        : 1.0.0
  ******************************************************************************
  * @detailed_description:
  * 本代码为自动控制实践综合实验代码综合，包含了回零、扫频、阶跃、跟随等功能
  * 
  * 环境说明：
  * 1. 本代码使用MotorControlWorkbench软件生成电机驱动代码，版本为5.2.0
  * 2. 本代码在STM32CubeMX 6.1.0版本下生成，使用了HAL库，版本为1.7.10
  * 3. 本代码使用了motorcontrol库，版本由MotorControlWorkbench确定
  * 4. 本代码在Keil MDK 5.37.0版本下编译通过
  * 
  * 功能说明：
  * 各个主要功能定义了相关的变量，其中布尔变量用于控制程序的执行流程和功能的开启
  * 各个变量的主要功能如下：
  * 1. 回零：find_home_flag             用于控制是否执行回零操作
  * 2. 扫频：sweep_identification_flag  用于控制是否执行扫频辨识操作
  * 3. 阶跃：step_test_flag             用于控制是否执行阶跃操作
  * 4. 跟随：follow_test_flag           用于控制是否执行扫频跟随操作
  * 功能的具体作用请结合实验指导书进行理解
  * 
  * 按键说明：
  * 按键设置为input模式，其中KEY1为workbench库默认启停按键，仍为外部中断模式
  * KEY2按键按下后，电机回零不可中断，回零完成后自动停止回零功能
  * KEY3~5按键按下后启动对应的功能，再次按下则停止对应的功能
  * 1. 按键KEY1：引脚-PE0，功能-电机启停
  * 2. 按键KEY2：引脚-PE1，功能-电机回零
  * 3. 按键KEY3：引脚-PE2，功能-扫频辨识
  * 4. 按键KEY4：引脚-PE3，功能-阶跃测试
  * 5. 按键KEY5：引脚-PE4，功能-扫频跟随
  * 可以结合RESET按键复位单片机
  * 
  * 编码器说明：    本程序使用TIM3作为编码器的计数器，计数器的计数范围为0~65535
  *                对于计数器溢出的情况，程序中已经做了处理，不需要额外处理
  *                可以查看Get_Encoder_Ruler_Count(void)函数的实现，其中有两种处理方式，可以自行选择
  * 
  * 串口说明：      本程序使用uart5作为与上位机通信的串口，波特率为115200
  * 
  * debug模式说明： 使用布尔变量 debug 控制是否开启debug模式
  *                debug模式下会输出一些调试信息，注意与matlab的通信会存在干扰
  * 
  * 控制器说明：    阶跃测试、扫频跟踪功能需要使用控制器
  *                本程序使用了离散化后的控制器，具体实现在discrete_control_func()函数中
  *                可以根据使用的电机系统辨识结果，设计控制器并修改该函数的实现
  ******************************************************************************
  * @detailed_description:
  * This code is the integrated code for the comprehensive experiment of automatic control practice, 
  * which includes functions such as return to zero, sweep, step, and follow
  * 
  * Environment description:
  * 1. This code uses MotorControlWorkbench software to generate motor drive code, version 5.2.0
  * 2. This code is generated under STM32CubeMX 6.1.0, using HAL library, version 1.7.10
  * 3. This code uses the motorcontrol library, the version is determined by MotorControlWorkbench
  * 4. This code is compiled through Keil MDK 5.37.0
  * 
  * Function description:
  * The main functions define related variables, among which boolean variables are used to control the execution flow of the program and the opening of functions
  * The main functions of each variable are as follows:
  * 1. Return to zero: find_home_flag is used to control whether to execute the return to zero operation
  * 2. Sweep: sweep_identification_flag is used to control whether to execute the sweep identification operation
  * 3. Step: step_test_flag is used to control whether to execute the step operation
  * 4. Follow: follow_test_flag is used to control whether to execute the sweep follow operation
  * The specific functions of the function, please refer to the experimental guide for understanding
  * 
  * Key description:
  * The key is set to input mode, where KEY1 is the workbench library default start/stop key, which is still external interrupt mode
  * After KEY2 is pressed, the motor cannot be interrupted by returning to zero, and the return to zero will stop automatically after completion
  * After KEY3~5 is pressed, the corresponding function is started, and then pressed again to stop the corresponding function
  * 1. KEY1: Pin-PE0, Function-Motor Start/Stop
  * 2. KEY2: Pin-PE1, Function-Motor Return to Zero
  * 3. KEY3: Pin-PE2, Function-Sweep Identification
  * 4. KEY4: Pin-PE3, Function-Step Test
  * 5. KEY5: Pin-PE4, Function-Sweep Follow
  * You can combine the RESET button to reset the microcontroller
  * 
  * Encoder description: This program uses TIM3 as the counter of the encoder, and the counter range is 0~65535
  * For the case of counter overflow, the program has been processed and no additional processing is required
  * You can check the implementation of the Get_Encoder_Ruler_Count(void) function, where there are two processing methods, you can choose
  * 
  * Serial port description: This program uses uart5 as the serial port for communication with the upper computer, and the baud rate is 115200
  * 
  * Debug mode description: Use the boolean variable debug to control whether to turn on the debug mode
  * Debug mode will output some debugging information, pay attention to the interference with matlab communication
  * 
  * Controller description: The step test and sweep tracking functions need to use the controller
  * This program uses the discretized controller, the specific implementation is in the discrete_control_func() function
  * According to the identification results of the motor system used, design the controller and modify the implementation of the function
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  unsigned int t_0;
  unsigned int t_01;
  float f0;
  float f1;
  float k;
  float p;
  float A;
}my_sweep_t;

typedef struct 
{ 
  uint8_t start_flag; /*帧的起始标志*/
  uint8_t frame_len; /*帧的长度信息*/ 
  uint8_t header_check; /*帧头的求和校验*/ 
  uint8_t data_buf[12]; /*数据长度，这里选择固定的数据长度*/ 
  uint8_t frame_check; /*帧头+数据的求和校验,用于接收方校验数据的完好性*/ 
}frame_matlab_t; /*数据帧结构体*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979323846

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// 编码器相关变量
// 记录电机当前位置
int32_t ENCODER_RULER_TIM_PERIOD=0;
float pos=0;

// 布尔值，判断电机是否到达左右两侧
bool encoder_ruler_left_limit_flag=false;
bool encoder_ruler_right_limit_flag=false;
// 两侧位置的标定值
float encoder_ruler_left_limit=0;
float encoder_ruler_right_limit=0;
// 中间位置的标定值
float encoder_ruler_middle=0;
// 布尔值，判断是否已经完成了两侧位置的标定
bool encoder_ruler_limit_calibrated=false;
// 常量值，电机到达中间位置的余量
const float middle_limit=2;

// debug模式
bool debug = false;   // 关闭debug模式
// bool debug = true; // 打开debug模式

// ========功能开关：回零========
// 执行回零操作
bool find_home_flag = false;
// 每次回零操作是否首先进行标定
bool re_calibrate_flag = true;
// bool re_calibrate_flag = false;

// ========功能开关：扫频========
// 扫频测试控制
bool sweep_identification_flag = false;
// 扫频是否需要重新初始化
bool sweep_reinit_flag = false;

// ========功能开关：阶跃========
// 阶跃测试控制
bool step_test_flag = false;
// 表明程序按下对应按键后首次执行阶跃函数
bool step_first_flag = true;

// ========功能开关：跟随========
// 扫频跟随测试
bool follow_test_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
float Get_Encoder_Ruler_Count(void);

// [GENERAL] 单个按键处理函数，进行消抖处理
// 传入要进行检测的按键的GPIO_Pin，返回按键是否按下
bool single_key_detect(uint16_t GPIO_Pin)
{
  // 按键按下
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_Pin) == GPIO_PIN_RESET)
  {
    // 按键按下，等待按键释放
    while(HAL_GPIO_ReadPin(GPIOE, GPIO_Pin) == GPIO_PIN_RESET);
    // 按键释放，返回true
    return true;
  }
  // 按键未按下，返回false
  else
  {
    return false;
  }
}

// [GENERAL] 按键处理函数
void keys_detect()
{
  // PE1已设置标签位GoHome
  // 若按键为PE1，find_home_flag置为true
  if(single_key_detect(Go_Home_Pin))
  {
    MC_StartMotor1();
    find_home_flag = true;
    // 重新标定编码器
    if(re_calibrate_flag)
    {
      encoder_ruler_left_limit_flag=false;
      encoder_ruler_right_limit_flag=false;
      encoder_ruler_limit_calibrated=false;
    }
  }
  // 若按键为PE2，sweep_test_flag进行翻转
  // 允许进行扫频辨识测试
  if(single_key_detect(Key_3_Pin))
  {
    MC_ProgramSpeedRampMotor1(0, 0);
    sweep_identification_flag = !sweep_identification_flag;
    // 重新进行扫频初始化
    sweep_reinit_flag = true;
    // 停止扫频时，将电机停止
    if(sweep_identification_flag==false)
    {
      MC_StopMotor1();
    }
    else
    {
      MC_StartMotor1();
    }
  }

  // 若按键为PE3，step_test_flag进行翻转
  // 允许进行阶跃测试
  if(single_key_detect(Key_4_Pin))
  {
    MC_StartMotor1();
    step_test_flag = !step_test_flag;
    // 当前一轮阶跃测试，函数首次执行
    step_first_flag = true;
    if(step_test_flag==false)
    {
      MC_StopMotor1();
    }
  }

  // 若按键为PE4，follow_test_flag进行翻转
  // 允许进行扫频跟随测试
  if(single_key_detect(Key_5_Pin))
  {
    MC_ProgramSpeedRampMotor1(0, 0);
    MC_StartMotor1();
    follow_test_flag = !follow_test_flag;
    if(follow_test_flag==false)
    {
      MC_StopMotor1();
    }
  }
}

// [TASK-2]
// 两侧限位器信号，判断电机是否到达两侧
// 其中PG1引脚为正限位，PG0引脚为负限位
void Judge_Limit_signal()
{
  // 电机到达左侧
  if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_1)==GPIO_PIN_RESET)
  {
    encoder_ruler_left_limit_flag=true;
    // [DEBUG]
    if (debug)
      printf("Left Limit!\r\n");
  }
  // 电机到达右侧
  else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_0)==GPIO_PIN_RESET)
  {
    encoder_ruler_right_limit_flag=true;
    // [DEBUG]
    if (debug)
      printf("Right Limit!\r\n");
  }
}

// [TASK-2] 寻找左右两侧的位置，进行位置标定
void Calibrate_Encoder_Ruler_Limit()
{
  // 当没有完成电机位置标定时，控制电机转动
  while(!encoder_ruler_limit_calibrated)
  {
    // 获取当前位置
    pos=Get_Encoder_Ruler_Count();
    // 两侧限位器信号，判断电机是否到达两侧，更新标志位
    Judge_Limit_signal();
    // 电机控制方法为给定电机一个速度和持续时间
    // 先标定左侧位置，再标定右侧位置
    if (!encoder_ruler_left_limit_flag)
    {
      MC_ProgramSpeedRampMotor1(50, 50);
      // 记录两侧位置的标定值
      encoder_ruler_left_limit=pos;
    }
    else if (!encoder_ruler_right_limit_flag)
    {
      MC_ProgramSpeedRampMotor1(-50, 50);
      // 记录两侧位置的标定值
      encoder_ruler_right_limit=pos;
    }
    else
    {
      // 电机到达两侧，停止电机
      MC_ProgramSpeedRampMotor1(0, 0);
      // 标定完成
      encoder_ruler_limit_calibrated=true;
    }
  }
  // 输出左右两侧位置的标定值
  // [DEBUG]
  if (debug)
  {
    printf("Encoder Ruler Left Limit:%f\r\n",encoder_ruler_left_limit);
    printf("Encoder Ruler Right Limit:%f\r\n",encoder_ruler_right_limit);
    // 输出标定完成
    printf("Encoder Ruler Limit Calibrated!\r\n");
    // 换行
    printf("\r\n");
  }
}

// [TASK-2] 计算中间位置
void Calibrate_Encoder_Ruler_Middle()
{
  // 计算中间位置的标定值
  encoder_ruler_middle=(encoder_ruler_left_limit+encoder_ruler_right_limit)/2;
  if(debug)
  {
    printf("Encoder Ruler Middle:%f\r\n",encoder_ruler_middle);
    // 输出标定完成
    printf("Encoder Ruler Middle Calibrated!\r\n");
  }
}

// [TASK-2] 控制电机速度，传入参数为电机位置，据此计算电机速度
void Control_Motor_Speed(float position)
{
  // [DEBUG]
  if (debug)
  {
    // 输出电机位置
    printf("Encoder Ruler Position:%f\r\n",position);
    // 输出中间位置
    printf("Encoder Ruler Middle:%f\r\n",encoder_ruler_middle);
    // 输出电机位置到中间位置的距离
    printf("Encoder Ruler Distance:%f\r\n",fabs(position-encoder_ruler_middle));
    // 换行
    printf("\r\n");
  }

  // 电机位置在中间位置的余量范围内，电机速度设为0
  if (fabs(position-encoder_ruler_middle)<middle_limit)
  {
    // [DEBUG]
    if (debug)
      printf("STOP MOTOR \r\n");
    MC_ProgramSpeedRampMotor1(0, 0);
    MC_StopMotor1();

    // 标记电机到达中间位置
    find_home_flag = false;
  }
  // 电机位置在中间位置的余量范围外
  else
  {
    // [DEBUG]
    if (debug)
      printf("MOTIVATE MOTOR \r\n");
    // 若电机位置在中间位置的左侧，电机速度为负
    if (position<encoder_ruler_middle)
    {
      MC_ProgramSpeedRampMotor1(50, 10);
    }
    // 若电机位置在中间位置的右侧，电机速度为正
    else
    {
      MC_ProgramSpeedRampMotor1(-50, 10);
    }
  }
}

// [TASK-3] 
// 初始化一个频率随着时间指数增加的正弦扫频信号的结构体
// 输入：unsigned int t_0 扫频函数的起始时刻， 单位ms
// 输入：unsigned int t_01 从t0到t1的时间间隔, 单位ms
// 输入：float f0 时刻t0对应的频率， 单位hz
// 输入：float f1 时刻t1对应的频率， 单位hz 
// 输入：float A 扫频信号的幅值
// 输出：int 0 = 成功返回0
int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A) 
{ 
  if ((t_01 == 0) || (f0 <=0.0f) || (f1 <= 0.0f) || (f0 == f1) || (A == 0) || (!sweep)) 
  {
    //非法入参
    return -1;
  }

  sweep->t_0 = t_0;
  sweep->t_01 = t_01;
  sweep->f0 = f0;
  sweep->f1 = f1;
  sweep->A = A;

  /* start add code here */ 
  /*计算指数函数的底数k，注意时间的单位要转换为ms*/ 
  sweep->k = exp(1.0 / (double)(sweep->t_01* 0.001) * log(sweep->f1 / sweep->f0));
  /*计算系数p, 注意单位转换*/
  sweep->p = 2 * PI * sweep->f0 / log(sweep->k);
  /* end add code here */

  return 0; 
}

// [TASK-3] 根据当前时间输出频率随着时间指数增加的正弦扫频信号
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now)
{
  float t = 0.0f; //相对时间 t 
  float y = 0.0f; //扫频信号 
  
  if (!sweep) 
  { 
    return 0.0f; /*非法入参*/ 
  }
  
  if (t_now < sweep->t_0) 
  { 
    return 0.0f; /*时间还未得到*/ 
  }
  
  t = (t_now - sweep->t_0) % sweep->t_01; /*通过求余操作实现，周期性扫频的过程*/
  t = t * 0.001f; /*将单位转换为 s*/
  
  /* start add your code here */
  y = sweep->A * sin(sweep->p * (pow(sweep->k, t) - 1));
  /* end add your code here */

  return y;
}

// [TASK-3] 计算校验和
uint8_t get_uint8_sum_check(uint8_t *data, int len) 
{ 
  int i = 0; 
  uint8_t sum = 0; 

  for (i = 0; i < len; i++) 
  {
    sum += data[i]; 
  }

  return sum; 
}

// [TASK-3] 向matlab发送三个浮点数
void send_data_2_matlab(float data1, float data2, float data3)
{ 
  frame_matlab_t frame = {0};

  // int i = 0; /*填充帧头*/ 
  frame.start_flag = 0xAA; 
  frame.frame_len = sizeof(frame); 
  frame.header_check = get_uint8_sum_check((uint8_t *)&frame, 2); /*填充数据*/

  memcpy((uint8_t *)&frame.data_buf[0], (uint8_t *)&data1, 4); 
  memcpy((uint8_t *)&frame.data_buf[4], (uint8_t *)&data2, 4); 
  memcpy((uint8_t *)&frame.data_buf[8], (uint8_t *)&data3, 4); /*计算数据求和值,用于接收方校验数据的完好性*/ 
  
  frame.frame_check = get_uint8_sum_check((uint8_t *)&frame, frame.frame_len-1); /*通过 串口5 发送到电脑 */ 
  
  HAL_UART_Transmit(&huart5, (uint8_t *)&frame,frame.frame_len,0xffff);   
}

// [TASK-3] 实现扫频辨识功能，该功能将被放置在main 函数的while(1)中运行
// bool is_reinit_sweep: 是否需要重新初始化扫频信号
void run_sweep_identification(bool is_reinit_sweep)
{
  static my_sweep_t sweep = {0};
  int16_t sweep_input = 0;
  int16_t sweep_output = 0;
  uint32_t sys_tick = 0;
  static uint32_t init_flag = 0; 
  static uint32_t last_sys_tick = 0; 
  // 若需要重新初始化扫频信号
  if (is_reinit_sweep == true) 
  { 
    init_flag = 0; 
  }

  // 频率在10s内，从0.5hz变化到10hz，幅度为1500 digit current
  uint32_t t_period_ms = 10*1000; //10s 
  float f0 = 0.5; 
  float f1 = 10; 
  float Amp = 1500.0f; 
  
  float time = 0.0f;
  sys_tick = HAL_GetTick(); //获取当前时刻，单位ms 
  time = 0.001f * sys_tick; //单位s

  /*进入的条件时回零成功，且按了K1运行, 就开始执行扫频辨识过程，
    注意find_home_flag是回零成功的标志位，是一个全局变量,要在外部实现这个标志变量*/
  // if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN)) {
  if ((sweep_identification_flag == true) && (MC_GetSTMStateMotor1() == RUN)) 
  {
    // 如果当前时刻发生了变化，这个条件每ms都会成立
    if (last_sys_tick != sys_tick)  
    {
      last_sys_tick = sys_tick;
      // 通过 % 把频率从1000hz降低到100hz，即每10ms发生一次变化
      if (sys_tick % 10 == 0) 
      {
          // 初始化扫频配置
          if (init_flag == 0) 
          {
              init_my_sweep(&sweep, sys_tick, t_period_ms, f0, f1, Amp);
              printf("sweep-init:k=%.5f,p=%.5f\r\n", (float)sweep.k,
                     (float)sweep.p);
              init_flag = 1;
          }

          // 获取正弦扫频信号
          sweep_input = (int16_t)run_my_sweep(&sweep, sys_tick);

          // 将正弦扫频信号输入到 ST MC SDK的力矩控制API中
          MC_ProgramTorqueRampMotor1(sweep_input, 0);  
          // 获取丝杆的转速信息，单位为0.1hz
          sweep_output = MC_GetMecSpeedAverageMotor1(); 
          // 把时间，input，output发送到matlab
          send_data_2_matlab(time, (float)sweep_input, (float)sweep_output);
          // [DEBUG]
          if(debug)
            printf("sweep-input:%d,sweep-output:%d\r\n", sweep_input, sweep_output);
      }
    }
  }
}

// [TASK-3] 控制函数实现
// ======================================================
// ====   根据电机实际辨识结果，设计控制器并进行离散化   ====
// ======================================================
int discrete_control_func(int32_t u0, float e1, float e0)
{
  return (int)(-0.6667*u0 + 3000*e1 - 2200*e0);
}

// [TASK-3] 实现阶跃测试函数，放在main的while(1)中进行阶跃测试
void function_step_test(void)
{
  /*1.定义局部变量和静态变量*/
  static int32_t u1 = 0;
  static int32_t u0 = 0;
  static float e1 = 0;
  static float e0 = 0;
  uint32_t sys_tick = 0;
  float fdk = 0;
  float Amp = 20;
  static float ref = 0;

  /*2.初始化你的控制器*/ 
  float time = 0.0f;
  sys_tick = HAL_GetTick();
  time = 0.001f * sys_tick;
  
  /*3.启动step测试功能条件满足*/
  // if ((find_home_flag==true)&&(MC_GetSTMStateMotor1()==RUN))
  if ((step_test_flag==true)&&(MC_GetSTMStateMotor1()==RUN))
  { 
    /*4.分频器确定当前tick为控制器运行的tick*/
    if (sys_tick % 10 == 0)
    { 
      if(step_first_flag == true)
      {
        /*5.获取光栅尺的当前位置 fdk (mm)*/ 
        fdk = Get_Encoder_Ruler_Count();

        /*6.指定阶跃的距离Amp统一定义为20mm*/ 
        Amp = 20;

        /*7.一次性计算当前参考值 ref = fdk+Amp*/ 
        ref = Amp + fdk;
        step_first_flag = false;
      }

      /*8.将ref作为你的位置控制器的输入命令，调用控制器*/ 
      e1 = ref - Get_Encoder_Ruler_Count();
      // u1 = (int)(-0.6667*u0 + 3000*e1 - 2200*e0);
      u1 = discrete_control_func(u0, e1, e0);
      
      // 限制u1的范围，处理饱和情况
      if(u1 > 4997) u1 = 4997;
      if(u1 < -4997) u1 = -4997;
      // 通过ST MC SDK的力矩控制API，将u1作为输入，控制电机
      MC_ProgramTorqueRampMotor1(u1, 0);
      // 更新e0和u0
      e0 = e1;
      u0 = u1;
      fdk = Get_Encoder_Ruler_Count();

      /*9.使用 send_data_2_matlab, 把时间，ref, fdk发送到matlab进行显示*/ 
      send_data_2_matlab(time, (float)ref, (float)fdk);
    } 
    else 
    {
      /*10. do some thing if need*/ 
    }
  }
  else 
  {
    /*11. do some thing if need*/ 
    // [DEBUG]
    if(debug)
    {
      printf("Can't run step test, please check the condition!\r\n");
    }
  }
}

// [TASK-3] 实现一个扫频测试函数，放在main的while(1)中进行扫频跟随测试，验证w位置闭环带宽
void function_follow_test(void)
{ 
  /*1.定义局部变量和静态变量*/
  static int32_t u1 = 0;
  static int32_t u0 = 0;
  static float e1 = 0;
  static float e0 = 0;
  uint32_t sys_tick = 0;
  float fdk;
  static float ref = 0;
  
  /*2.初始化你的控制器*/
  float time = 0.0f;
  sys_tick = HAL_GetTick();
  time = 0.001f * sys_tick;
  
  /*3.初始化你的正弦信号发生器幅值为20mm, 频率在10s内从1hz变化到2hz*/
  //10s 1hz->2hz
  static float w = 1*2*3.1415;
  float Amp = 20;

  /*4.启动扫频跟随测试功能条件满足*/
  // if ((find_home_flag==1)&&(MC_GetSTMStateMotor1()==RUN)) 
  if ((follow_test_flag)&&(MC_GetSTMStateMotor1()==RUN)) 
  { 
    /*5.分频器确定当前tick为控制器运行的tick*/
    if (sys_tick % 10 == 0) 
    { 
      /*6.获取光栅尺的当前位置 fdk (mm)*/
      fdk = Get_Encoder_Ruler_Count();
      
      /*7.每个tick都获取最新的扫频信号输出，作为当前参考值 ref = sweep(now)*/
      ref = Amp*sin(w*time);
      
      /*8.将ref作为你的位置控制器的输入命令，调用控制器*/
      e1 = ref - fdk;
      // u1 = (int)(-0.6667*u0 + 3000*e1 - 2200*e0);
      u1 = discrete_control_func(u0, e1, e0);

      // 限制u1的范围，处理饱和情况
      if(u1 > 4997) u1 = 4997;
      if(u1 < -4997) u1 = -4997;
      // 通过ST MC SDK的力矩控制API，将u1作为输入，控制电机
      MC_ProgramTorqueRampMotor1(u1, 0);
      // 更新e0和u0
      e0 = e1;
      u0 = u1;
      fdk = Get_Encoder_Ruler_Count();
      /*9.使用 send_data_2_matlab, 把时间，ref, fdk发送到matlab进行显示*/ 
      send_data_2_matlab(time, (float)ref, (float)fdk);
    }
    else
    { 
      /*10. do some thing if need*/
    } 
  } 
  else 
  {
    /*11. do some thing if need*/ 
    // [DEBUG]
    if(debug)
    {
      printf("Can't run follow test, please check the condition!\r\n");
    }
  } 
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 
	MC_AlignEncoderMotor1();
  HAL_Delay(100);

  // 启动电机
  MC_StartMotor1();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 按键检测
    keys_detect();
    // 获取当前位置
    pos=Get_Encoder_Ruler_Count();
    
    // 电机回零
    if (find_home_flag==true)
    {
      // 电机位置标定
      Calibrate_Encoder_Ruler_Limit();
      // 计算中间位置
      Calibrate_Encoder_Ruler_Middle();
      // 控制电机速度，使得电机回零
      Control_Motor_Speed(pos);
    }
    // 进行扫频辨识
    if(sweep_identification_flag==true)
    {
      run_sweep_identification(sweep_reinit_flag);
      sweep_reinit_flag = false;
    }
    // 进行阶跃测试
    if(step_test_flag==true)
    {
      function_step_test();
    }
    // 进行扫频跟随测试
    if(follow_test_flag==true)
    {
      function_follow_test();
    }
	  // printf("%.3f \r\n",pos);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM8_BRK_TIM12_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_URS_ENABLE(&htim3);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim8.Init.RepetitionCounter = (REP_COUNTER);
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : Key_4_Pin Key_3_Pin Go_Home_Pin Key_5_Pin */
  GPIO_InitStruct.Pin = Key_4_Pin|Key_3_Pin|Go_Home_Pin|Key_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM_P_Pin LIM_N_Pin */
  GPIO_InitStruct.Pin = LIM_P_Pin|LIM_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// // GPIO外部中断回调函数
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
// }

// 初始上电时电机位置为0，两侧距离用不同符号表示
// 经过测试，电机在整个导轨上的运动范围不会超过限制
float Get_Encoder_Ruler_Count(void)
{
  // 使用int32_t类型，由于实际计数值为16位，Value始终为正数
	int32_t Value = __HAL_TIM_GET_COUNTER(&htim3);
  
  float CaptureNumber=0;
	float ruler_pos=0.0f;

  // 16位数的范围位0~65535，当数值大于32768时，说明计数值溢出，需要减去65536
  CaptureNumber = (Value > 32768) ? (Value - 65536) : (Value);
  // 将距离进行转换，单位为mm
	ruler_pos = (float)(CaptureNumber*0.005f);

  // [DEBUG]
  if(debug)
  {
    printf("Value:%d\r\n",Value);
    printf("ruler_pos:%f\r\n",ruler_pos);
  }

	return ruler_pos;
  // 实际使用int16_t类型时，可以正确显示负数
  // 不需要进行额外处理，但是不想改了，就写在注释里面吧
  // int16_t test_value = __HAL_TIM_GET_COUNTER(&htim3);
  // printf("test_value:%d\r\n",test_value);
  // ruler_pos = (float)(test_value*0.005f);
  // return ruler_pos;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
