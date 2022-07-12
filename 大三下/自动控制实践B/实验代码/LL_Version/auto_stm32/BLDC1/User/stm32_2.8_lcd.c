/**
  ******************************************************************************
  * @file    stm3210c_eval_lcd.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file includes the LCD driver for AM-240320L8TNQW00H (LCD_ILI9320)
  *          Liquid Crystal Display Module of STM3210C-EVAL board.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_2.8_lcd.h"
#include "fonts.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @defgroup STM3210C_EVAL_LCD 
  * @brief This file includes the LCD driver for AM-240320L8TNQW00H (LCD_ILI9320)
  *        Liquid Crystal Display Module of STM3210C-EVAL board.
  * @{
  */ 

/** @defgroup STM3210C_EVAL_LCD_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LCD_Private_Defines
  * @{
  */ 
#define START_BYTE      0x70
#define SET_INDEX       0x00
#define READ_STATUS     0x01
#define LCD_WRITE_REG   0x02
#define LCD_READ_REG    0x03
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LCD_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LCD_Private_Variables
  * @{
  */ 
  /* Global variables to set the written text color */
vu16 TextColor = 0x0000, BackColor = 0xFFFF;
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LCD_Private_FunctionPrototypes
  * @{
  */ 
#ifndef USE_Delay
static void delay(vu32 nCount);
#endif /* USE_Delay*/
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LCD_Private_Functions
  * @{
  */ 

void  Delay (u32 nCount)
{
    volatile  int  i;


    for (i = 0; i < nCount ; i++) {
        ;
    }
}

/**
  * @brief  Setups the LCD.
  * @param  None
  * @retval None
  */
void LCD_Setup(void)
{
 LCD_CtrlLinesConfig();
  
  LCD_GPIOConfig();
	_delay_(10); /* Delay 50 ms */
		
  LCD_WriteReg(R0,  0x0001); 
  LCD_WriteReg(R1,  0x0100); 
  LCD_WriteReg(R2,  0x0700); 
  LCD_WriteReg(R3,  0x1030); 
  LCD_WriteReg(R4,  0x0000); 
  LCD_WriteReg(R8,  0x0202); 
  LCD_WriteReg(R9,  0x0000); 
  LCD_WriteReg(R10, 0x0000); 
  LCD_WriteReg(R12, 0x0000);
  LCD_WriteReg(R13, 0x0000); 
  LCD_WriteReg(R15, 0x0000); 

 
  LCD_WriteReg(R16, 0x0000); 
  LCD_WriteReg(R17, 0x0000); 
  LCD_WriteReg(R18, 0x0000); 
  LCD_WriteReg(R19, 0x0000); 
  _delay_(10);                 
  LCD_WriteReg(R16, 0x17B0); 
  LCD_WriteReg(R17, 0x0137); 
  _delay_(10);                  
  LCD_WriteReg(R18, 0x0139);       
  _delay_(10);                 
  LCD_WriteReg(R19, 0x1d00); 
  LCD_WriteReg(R41, 0x0013); 
  _delay_(10);               
  LCD_WriteReg(R32, 0x0000); 
  LCD_WriteReg(R33, 0x0000); 

  LCD_WriteReg(R48, 0x0006);
  LCD_WriteReg(R49, 0x0101);
  LCD_WriteReg(R50, 0x0003);
  LCD_WriteReg(R53, 0x0106);
  LCD_WriteReg(R54, 0x0b02);
  LCD_WriteReg(R55, 0x0302);
  LCD_WriteReg(R56, 0x0707);
  LCD_WriteReg(R57, 0x0007);
  LCD_WriteReg(R60, 0x0600);
  LCD_WriteReg(R61, 0x020b);
 
  LCD_WriteReg(R80, 0x0000); 
  LCD_WriteReg(R81, 0x00EF); 
  LCD_WriteReg(R82, 0x0000); 
  LCD_WriteReg(R83, 0x013F); 
  LCD_WriteReg(R96,  0xa700); 
  LCD_WriteReg(R97,  0x0001); 
  LCD_WriteReg(R106, 0x0000);

  LCD_WriteReg(R128, 0x0000);
  LCD_WriteReg(R129, 0x0000);
  LCD_WriteReg(R130, 0x0000);
  LCD_WriteReg(R131, 0x0000);
  LCD_WriteReg(R132, 0x0000);
  LCD_WriteReg(R133, 0x0000);

  LCD_WriteReg(R144, 0x0010);
  LCD_WriteReg(R146, 0x0000);
  LCD_WriteReg(R147, 0x0003);
  LCD_WriteReg(R149, 0x0110);
  LCD_WriteReg(R151, 0x0000);
  LCD_WriteReg(R152, 0x0000);
    
  LCD_WriteReg(R3, 0x1018);
  LCD_WriteReg(R7, 0x0173);   
}

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval None
  */
void STM32_LCD_Init(void)
{
  /* Setups the LCD */
  LCD_Setup();
}


/**
  * @brief  Sets the Text color.
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(vu16 Color)
{
  TextColor = Color;
}


/**
  * @brief  Sets the Background color.
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetBackColor(vu16 Color)
{
  BackColor = Color;
}


/**
  * @brief  Clears the selected line.
  * @param  Line: the Line to be cleared.
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @retval None
  */
void LCD_ClearLine(u8 Line)
{
  LCD_DisplayStringLine(Line, "                    ");
}


/**
  * @brief  Clears the hole LCD.
  * @param  Color: the color of the background.
  * @retval None
  */
void LCD_Clear(u16 Color)
{
  u32 index = 0;
  
  LCD_SetCursor(0x00, 0x013F); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 76800; index++)
  {
    LCD_WriteRAM(Color);
  }
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET); 	  
}


/**
  * @brief  Sets the cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
void LCD_SetCursor(u8 Xpos, u16 Ypos)
{
 LCD_WriteReg(R32, Xpos);
 LCD_WriteReg(R33, Ypos);
}


/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChar(u8 Xpos, u16 Ypos, const u16 *c)
{
  u32 index = 0, i = 0;
  u8 Xaddress = 0;
   
  Xaddress = Xpos;
  
  LCD_SetCursor(Xaddress, 319-Ypos);
  
  for(index = 0; index < 24; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < 16; i++)
    {
      if((c[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }   
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET); 
    Xaddress++;
    LCD_SetCursor(Xaddress, 319-Ypos);
  }
}



/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(u8 Line, u16 Column, u8 Ascii)
{
  Ascii -= 32;
  LCD_DrawChar(Line, Column, &ASCII_Table[Ascii * 24]);
}


/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  *ptr: pointer to string to display on LCD.
  * @retval None
  */
void LCD_DisplayStringLine(u8 Line, u8 *ptr)
{
  u32 i = 0;
  u16 refcolumn =0;
  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (i < 20))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line*FONT_HEIGHT, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn += FONT_WIDTH;
    /* Point on the next character */
    ptr++;
    /* Increment the character counter */
    i++;
  }
}


/**
  * @brief  Sets a display window
  * @param  Xpos: specifies the X buttom left position.
  * @param  Ypos: specifies the Y buttom left position.
  * @param  Height: display window height.
  * @param  Width: display window width.
  * @retval None
  */
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width)
{ 
  /* Horizontal GRAM Start Address */
  if(Xpos >= Height)
  {
    LCD_WriteReg(R80, (Xpos - Height + 1));
  }
  else
  {
    LCD_WriteReg(R80, 0);
  }
  /* Horizontal GRAM End Address */
  LCD_WriteReg(R81, Xpos);
  /* Vertical GRAM Start Address */
  if(Ypos >= Width)
  {
    LCD_WriteReg(R82, (Ypos - Width + 1));
  }  
  else
  {
    LCD_WriteReg(R82, 0);
  }
  /* Vertical GRAM End Address */
  LCD_WriteReg(R83, Ypos);
  LCD_SetCursor(Xpos, Ypos);
}


/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval None
  */
void LCD_WindowModeDisable(void)
{
  LCD_SetDisplayWindow(239, 0x13F, 240, 320);
  LCD_WriteReg(R3, 0x1018);    
}


/**
  * @brief  Displays a line.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: line length.
  * @param  Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None
  */
void LCD_DrawLine(u8 Xpos, u16 Ypos, u16 Length, u8 Direction)
{
  u32 i = 0;
  
  LCD_SetCursor(Xpos, 319-Ypos);
  if(Direction == Horizontal)
  { 
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET);
  }
  else
  {
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAMWord(TextColor);
      Xpos++;
      LCD_SetCursor(Xpos, Ypos);
    }
  }
}


/**
  * @brief  Displays a rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void LCD_DrawRect(u8 Xpos, u16 Ypos, u8 Height, u16 Width)
{
  LCD_DrawLine(Xpos, Ypos, Width, Horizontal);
  LCD_DrawLine((Xpos + Height), Ypos, Width, Horizontal);
  
  LCD_DrawLine(Xpos, 319-Ypos, Height, Vertical);
  LCD_DrawLine(Xpos, (319-(Ypos + Width - 1)), Height, Vertical);
}


/**
  * @brief  Displays a circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawCircle(u8 Xpos, u16 Ypos, u16 Radius)
{
  s32  D;/* Decision Variable */ 
  u32  CurX;/* Current X Value */
  u32  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, 319-Ypos + CurY);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos + CurX, 319-Ypos - CurY);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos - CurX, 319-Ypos + CurY);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos - CurX, 319-Ypos - CurY);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos + CurY,319- Ypos + CurX);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos + CurY, 319-Ypos - CurX);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos - CurY, 319-Ypos + CurX);
    LCD_WriteRAMWord(TextColor);
    LCD_SetCursor(Xpos - CurY, 319-Ypos - CurX);
    LCD_WriteRAMWord(TextColor);
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
  * @brief  Displays a monocolor picture.
  * @param  Pict: pointer to the picture array.
  * @retval None
  */
void LCD_DrawMonoPict(const u32 *Pict)
{
  u32 index = 0, i = 0;
  LCD_SetCursor(0, 319); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 2400; index++)
  {
    for(i = 0; i < 32; i++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET);
}

#ifdef USE_LCD_DrawBMP

/**
  * @brief  Displays a bitmap picture loaded in the SPI Flash.
  * @param  BmpAddress: Bmp picture address in the SPI Flash.
  * @retval None
  */
void LCD_DrawBMP(const u16 *BmpAddress)
{
  u32 i = 0, size = 0;
  /* Read bitmap size */
  size = BmpAddress[1] | (BmpAddress[2] << 16);
  /* get bitmap data address offset */
  i = BmpAddress[5] | (BmpAddress[6] << 16);
  size = (size - i)/2;
  BmpAddress += i/2;
  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  LCD_WriteReg(R3, 0x1008);
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  /* Read bitmap data from SPI Flash and send them to LCD */
  for(i = 0; i < size; i++)
  {
    LCD_WriteRAM(BmpAddress[i]);
  }
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
  LCD_WriteReg(R3, 0x1018);
}
#endif

/**
  * @brief  Reset LCD control line(/CS) and Send Start-Byte
  * @param  Start_Byte: the Start-Byte to be sent
  * @retval None
  */
void LCD_nCS_StartByte(u8 Start_Byte)
{
  
 u16 x;
 x=Start_Byte;
	
	HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_RESET);  
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET);
 
   
	LCD_PORT_PORT->ODR = x;

  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET); 	
}

/**
  * @brief  Writes index to select the LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void LCD_WriteRegIndex(u8 LCD_Reg)
{
  u16 x;
  x=LCD_Reg;
 // x=x<<8;
 // LCD_nCS_StartByte(START_BYTE | SET_INDEX); 
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET);  


	HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_RESET);  
  LCD_PORT_PORT->ODR = 0;
  //Delay(10);		
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET); 

  LCD_PORT_PORT->ODR =x;
 // Delay(10);		
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET); 	   
  
	HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_SET); 

  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET);	
}





/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
void LCD_WriteRAM_Prepare(void)
{
  LCD_WriteRegIndex(R34); /* Select GRAM Reg */
  /* Reset LCD control line(/CS) and Send Start-Byte */
 // LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);
	
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET);
}


/**
  * @brief  Writes 1 word to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAMWord(u16 RGB_Code)
{
  LCD_WriteRAM_Prepare();
  LCD_WriteRAM(RGB_Code);
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET);
}


/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{
  /* Write 16-bit Index (then Write Reg) */
  LCD_WriteRegIndex(LCD_Reg);
  /* Write 16-bit Reg */
  /* Reset LCD control line(/CS) and Send Start-Byte */
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_RESET);	
   
  HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_SET); 	
	LCD_PORT_PORT->ODR =LCD_RegValue>>8;
 // Delay(5);		
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
  //Delay(10);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET); 
	LCD_PORT_PORT->ODR =LCD_RegValue;
 // Delay(10);		
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
  //Delay(10);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET); 
	
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, GPIO_PIN_SET);		
}


/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(u16 RGB_Code)
{
	HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_SET);  
	LCD_PORT_PORT->ODR = RGB_Code>>8;
	
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET);  

	LCD_PORT_PORT->ODR = RGB_Code;
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET);  
}


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void LCD_PowerOn(void)
{ 
  /* Power On sequence ---------------------------------------------------------*/
  LCD_WriteReg(R16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(R17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
  LCD_WriteReg(R18, 0x0000); /* VREG1OUT voltage */
  LCD_WriteReg(R19, 0x0000); /* VDV[4:0] for VCOM amplitude */
  _delay_(20);                 /* Dis-charge capacitor power voltage (200ms) */
  LCD_WriteReg(R16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(R17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(R18, 0x0139); /* VREG1OUT voltage */
  _delay_(5);                  /* delay 50 ms */
  LCD_WriteReg(R19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
  LCD_WriteReg(R41, 0x0013); /* VCM[4:0] for VCOMH */
  _delay_(5);                  /* delay 50 ms */
  LCD_WriteReg(R7, 0x0173);  /* 262K color and display ON */
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOn(void)
{
  /* Display On */
  LCD_WriteReg(R7, 0x0173); /* 262K color and display ON */
 
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOff(void)
{
  /* Display Off */
  LCD_WriteReg(R7, 0x0);
}

/**
  * @brief  Configures LCD control lines in Output Push-Pull mode.
  * @param  None
  * @retval None
  */
void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
   unsigned int i;
  
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	
	GPIO_InitStruct.Pin = LCD_NCS_PIN;
  HAL_GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStruct);
	
  
	GPIO_InitStruct.Pin = LCD_RST_PIN;
  HAL_GPIO_Init(LCD_RST_GPIO_PORT, &GPIO_InitStruct);
  
	
  HAL_GPIO_WritePin(LCD_RST_GPIO_PORT,LCD_RST_PIN,GPIO_PIN_RESET);
  for(i=0;i<5000;i++);	
  HAL_GPIO_WritePin(LCD_RST_GPIO_PORT,LCD_RST_PIN,GPIO_PIN_SET);
}

/**
  * @brief  Sets or reset LCD control lines.
  * @param  GPIOx: where x can be B or D to select the GPIO peripheral.
  * @param  CtrlPins: the Control line. This parameter can be:
  *     @arg LCD_NCS_PIN: Chip Select pin
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, u16 CtrlPins, GPIO_PinState BitVal)
{
  HAL_GPIO_WritePin(GPIOx,CtrlPins,BitVal);
}


/**
  * @brief  Configures the LCD_SPI interface.
  * @param  None
  * @retval None
  */
/**
  * @brief  Configures the LCD_SPI interface.
  * @param  None
  * @retval None
  */
void LCD_GPIOConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock */  
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
	
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	
	GPIO_InitStruct.Pin = LCD_WR_PIN;
  HAL_GPIO_Init(LCD_WR_PORT, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = LCD_RD_PIN;
  HAL_GPIO_Init(LCD_RD_PORT, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = LCD_RS_PIN;
  HAL_GPIO_Init(LCD_RS_PORT, &GPIO_InitStruct);
   
  
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LCD_RS_PORT,LCD_RS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RD_PORT,LCD_RD_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_WR_PORT,LCD_WR_PIN,GPIO_PIN_SET);
}

/******************************************************************************/
void LCD_Draw_NUM(u8 Xpos, u16 Ypos, u32 da )
{
  u8 x;
  //da=12345;
  x=da/10000;  
  LCD_DisplayChar(Xpos,320-Ypos,x+0x30);	 
  x=da/1000%10;
  LCD_DisplayChar(Xpos,320-(Ypos-16),x+0x30);
  x=da/100%10;
  LCD_DisplayChar(Xpos,320-(Ypos-32),x+0x30);
  x=da/10%10;
  LCD_DisplayChar(Xpos,320-(Ypos-48),x+0x30);
  x=da%10;
  LCD_DisplayChar(Xpos,320-(Ypos-64),x+0x30);	  
}

void LCD_ShowNum(uint8_t x,uint16_t y,uint16_t data)
{ //��ʾ������תΪ�ַ���ʾ�����ϣ�X,YΪ����
LCD_DisplayChar(x,y,data/10000+48); 
LCD_DisplayChar(x,(y+25),data%10000/1000+48);   // %10000
LCD_DisplayChar(x,(y+50),data%1000/100+48); 
LCD_DisplayChar(x,(y+75),data%100/10+48);	 
LCD_DisplayChar(x,(y+100),data%10+48);
}

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(vu32 nCount)
{
  vu32 index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/
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
