/**
  ******************************************************************************
  * @file    stm3210c_eval_lcd.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file contains all the functions prototypes for the lcd firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_LCD_H
#define __STM32_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

	extern  vu16 TextColor,BackColor ;

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM3210C_EVAL_LCD
  * @{
  */


/** @defgroup STM3210C_EVAL_LCD_Exported_Types
  * @{
  */
/**
  * @}
  */



/** @defgroup STM3210C_EVAL_LCD_Exported_Constants
  * @{
  */

/**
 * @brief Uncomment the line below if you want to use LCD_DrawBMP function to
 *        display a bitmap picture on the LCD. This function assumes that the bitmap
 *        file is loaded in the SPI Flash (mounted on STM3210C-EVAL board), however
 *        user can tailor it according to his application hardware requirement.
 */
/*#define USE_LCD_DrawBMP*/

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).
 */
/* #define USE_Delay */

#ifdef USE_Delay
#include "main.h"

  #define _delay_     Delay  /* !< User can provide more timing precise _delay_ function
                                   (with 10ms time base), using SysTick for example */
#else
  #define _delay_     delay      /* !< Default _delay_ function with less precise timing */
#endif

/** 
  * @brief  LCD Control pins  
  */ 

#define LCD_RST_PIN            GPIO_PIN_5                  
#define LCD_RST_GPIO_PORT      GPIOB                      
#define LCD_RST_GPIO_CLK       RCC_AHB1Periph_GPIOB  

#define LCD_NCS_PIN            GPIO_PIN_12                  
#define LCD_NCS_GPIO_PORT      GPIOC                       
#define LCD_NCS_GPIO_CLK       RCC_AHB1Periph_GPIOC  

#define LCD_WR_PIN       GPIO_PIN_15           
#define LCD_WR_PORT      GPIOD                       
#define LCD_WR_CLK       RCC_AHB1Periph_GPIOD  
          
#define LCD_RD_PIN       GPIO_PIN_9          
#define LCD_RD_PORT      GPIOD                      
#define LCD_RD_CLK       RCC_AHB1Periph_GPIOD    
           
#define LCD_RS_PIN       GPIO_PIN_7         
#define LCD_RS_PORT      GPIOD                      
#define LCD_RS_CLK       RCC_AHB1Periph_GPIOD   
                    
#define LCD_PORT_PORT      GPIOG                                
#define LCD_PORT_CLK       RCC_AHB1Periph_GPIOG 

/**
  * @brief  LCD Registers
  */
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5

/**
  * @brief  LCD color
  */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0
/*
#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216	*/


#define Line0          0
#define Line1          1
#define Line2          2
#define Line3          3
#define Line4          4
#define Line5          5
#define Line6          6
#define Line7          7
#define Line8          8
#define Line9          9

#define Horizontal     0x00
#define Vertical       0x01

/*************		New Macro		 *************/
#define FONT_WIDTH     16
#define FONT_HEIGHT    24

#define Max_LCDXPostion 	(240-1)
#define Max_LCDYPostion  	(320-1)

/**
  * @}
  */

/** @defgroup STM3210C_EVAL_LCD_Exported_Macros
  * @{
  */
/**
  * @}
  */



/** @defgroup STM3210C_EVAL_LCD_Exported_Functions
  * @{
  */
void LCD_Setup(void);
void STM32_LCD_Init(void);
void LCD_SetTextColor(volatile u16 Color);
void LCD_SetBackColor(volatile u16 Color);
void LCD_ClearLine(u8 Line);
void LCD_Clear(u16 Color);
void LCD_SetCursor(u8 Xpos, u16 Ypos);
void LCD_DrawChar(u8 Xpos, u16 Ypos, const u16 *c);
void LCD_DisplayChar(u8 Line, u16 Column, u8 Ascii);
void LCD_DisplayStringLine(u8 Line, u8 *ptr);
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(u8 Xpos, u16 Ypos, u16 Length, u8 Direction);
void LCD_DrawRect(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_DrawCircle(u8 Xpos, u16 Ypos, u16 Radius);
void LCD_DrawMonoPict(const u32 *Pict);
#ifdef USE_LCD_DrawBMP
//void LCD_DrawBMP(u32 BmpAddress);
void LCD_DrawBMP(const u16 *BmpAddress);
#endif

void LCD_nCS_StartByte(u8 Start_Byte);
void LCD_WriteRegIndex(u8 LCD_Reg);
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAMWord(u16 RGB_Code);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM(u16 RGB_Code);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

void LCD_CtrlLinesConfig(void);
void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, u16 CtrlPins, GPIO_PinState BitVal);
void LCD_GPIOConfig(void);

void LCD_ShowNum(uint8_t x,uint16_t y,uint16_t data);
void LCD_Draw_NUM(u8 Xpos, u16 Ypos, u32 da );

#ifdef __cplusplus
}
#endif

#endif 
/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
