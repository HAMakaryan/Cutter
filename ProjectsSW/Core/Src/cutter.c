/*
 * cutter.c
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "cutter.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;

uint8_t lcd_buf_length = 0;
uint8_t lcd_write_pnt = 0;
uint8_t lcd_read_pnt = 0;
uint8_t lcd_timeout = 0;
uint8_t completed = 1;
uint8_t data_arr[4];
uint16_t lcd_ring_buffer[LCD_BUF_SIZE];

/**
  * @brief	Sends data to LCD with blocking mode.
  * @param	LCD address
  * @param	Data to be sent
  * @param	RS bit 0/1
  * @retval	None
  */
void LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags)
{
    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
}

/**
  * @brief	Sends a command to LCD with blocking mode.
  * @param	LCD address
  * @param	Command to be sent
  * @retval	None
  */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd)
{
    LCD_SendInternal(lcd_addr, cmd, 0);
}

/**
  * @brief	Sends a data to LCD with blocking mode.
  * @param	LCD address
  * @param	Data to be sent
  * @retval	None
  */
void LCD_SendData(uint8_t lcd_addr, uint8_t data)
{
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

/**
  * @brief	Sends a string to LCD with blocking mode.
  * @param	LCD address
  * @param	String to be sent
  * @retval	None
  */
void LCD_SendString(uint8_t lcd_addr, char *str)
{
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

/**
  * @brief	Initializes the LCD(16x4) module using I2C peripheral.
  * @param	lcd address
  * @retval	None
  */
void LCD_Init(uint8_t lcd_addr)
{
	/* Initializes the following
	* Function Set
	* Entry mode
	* Display on/off
	* Clear Display
	* Needed to declare LCD_Write_Command() function
	  to send command to the LCD
	  */
	/*// 4-bit mode, 2 lines, 5x7 format
	LCD_SendCommand(lcd_addr, 0x30); //0b00110000
	// display & cursor home (keep this!)
	LCD_SendCommand(lcd_addr, 0x02); //0b00000010
	// display on, right shift, underline off, blink off
	LCD_SendCommand(lcd_addr, 0x0C); //0b00001100
	// clear display (optional here)
	LCD_SendCommand(lcd_addr, 0x01);*/

	HAL_Delay(50);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(5);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(1);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(10);
	LCD_SendCommand(lcd_addr, 0x02);	//4 bit mode
	HAL_Delay(10);

	LCD_SendCommand(lcd_addr, 0x28);      // configuring LCD as 2 line 5x7 matrix
	LCD_SendCommand(lcd_addr, 0x0E);      // Display on, Cursor on
	LCD_SendCommand(lcd_addr, 0x01);      // Clear Display Screen
	LCD_SendCommand(lcd_addr, 0x06);      // Increment cursor, no shift
}

/**
  * @brief	Checks buffer is full or no.
  * @param	None
  * @retval	0/1
  */
uint8_t Full(uint8_t buff_size)
{
  if (buff_size == LCD_BUF_SIZE)
  {
    return 1;
  } else {
    return 0;
  }
}

/**
  * @brief	Checks buffer is empty or no.
  * @param	None
  * @retval	0/1
  */
uint8_t Empty(uint8_t buff_size)
{
  if (buff_size == 0)
  {
    return 1;
  } else {
    return 0;
  }
}

/**
  * @brief	Writes data to buffer
  * @param	Data
  * param 	Size of data
  * @retval	None
  */
void LCD_Write_Buffer(uint16_t *data, uint8_t size)
{
	for (int i = 0; i < size; ++i)
	{
	  if (!Full(lcd_buf_length))
	  {
		  lcd_ring_buffer[lcd_write_pnt] = data[i];
		  lcd_write_pnt++;
		  lcd_buf_length++;
		  if (lcd_write_pnt == LCD_BUF_SIZE)
		  {
			  lcd_write_pnt = 0;
		  }
	  }
	}
}

/**
  * @brief	Reads regular data from the buffer
  * @param	Data
  * param 	None
  * @retval	Data
  */
uint16_t LCD_Read_Buffer()
{
  uint16_t data = lcd_ring_buffer[lcd_read_pnt];

  lcd_read_pnt++;
  lcd_buf_length--;

  if (lcd_read_pnt == LCD_BUF_SIZE)
  {
	  lcd_read_pnt = 0;
  }
  return data;
}

/**
  * @brief	Displays a char on the LCD with non-blocking mode.
  * @retval None
  */
void LCD_Write(uint8_t lcd_addr)
{
	//every ms
	if (lcd_timeout == LCD_TIMEOUT)
	{
		lcd_timeout = 0;

		if (!Empty(lcd_buf_length))
		{
			uint16_t data = LCD_Read_Buffer();
			uint8_t up = data & 0xF0;
			uint8_t lo = (data << 4) & 0xF0;
			uint8_t rs = 0;

			rs = PIN_RS;

			//if data
			if ((data & LCD_DATA_MASK) == LCD_DATA_MASK)
			{
				data_arr[0] = up|rs|BACKLIGHT|PIN_EN;
				data_arr[1] = up|rs|BACKLIGHT;
				data_arr[2] = lo|rs|BACKLIGHT|PIN_EN;
				data_arr[3] = lo|rs|BACKLIGHT;
			//if command
			} else {
				rs = 0;
				data_arr[0] = up|rs|BACKLIGHT|PIN_EN;
				data_arr[1] = up|rs|BACKLIGHT;
				data_arr[2] = lo|rs|BACKLIGHT|PIN_EN;
				data_arr[3] = lo|rs|BACKLIGHT;
			}

			if (completed == 1)
			{
				completed = 0;
				//HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, 4, 1000);

				HAL_I2C_Master_Transmit_IT(&hi2c1, lcd_addr, data_arr, 4);

				/*for (int i = 0; i < 4; ++i)
				{
					int  num = 32424;
					char hex[10];

					sprintf(hex, "%02x\n\r", data_arr[i]);
					HAL_UART_Transmit(&huart3, hex, 10, 0xFFFF);
				}*/
			}
		}
	}
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	completed = 1;
	/* Prevent unused argument(s) compilation warning */
	//UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ErrorCallback could be implemented in the user file
   */
}

///////////////////////////////////////////////////////////////////////////////
uint8_t keypad_timeout = 0;
uint8_t pos[ROW_SIZE] = {1, 2, 4, 8};
uint8_t keypad_buffer[KEYPAD_BUF_SIZE];
uint8_t keypad_buf_length = 0;
uint8_t keypad_wr_pnt = 0;
uint8_t keypad_rd_pnt = 0;

GPIO_TypeDef* row_gpio_port[ROW_SIZE] = {Row0_GPIO_Port, Row1_GPIO_Port,
										Row2_GPIO_Port, Row3_GPIO_Port};
GPIO_TypeDef* col_gpio_port[COL_SIZE] = {Col0_GPIO_Port, Col1_GPIO_Port,
										Col2_GPIO_Port, Col3_GPIO_Port};
uint16_t row_gpio_pin[ROW_SIZE] = {Row0_Pin, Row1_Pin, Row2_Pin, Row3_Pin};
uint16_t col_gpio_pin[COL_SIZE] = {Col0_Pin, Col1_Pin, Col2_Pin, Col3_Pin};

uint8_t state = IDLE;
uint8_t new_pressed_key = 0;
uint8_t old_pressed_key = 0;
uint8_t debounce = 0;
uint8_t row_key = 0;
uint8_t col_key = 0;
/**
  * @brief	Initializes the keypad(4x4).
  * @param	None
  * @retval None
  */
void Keypad_Init()
{
	//set all rows and columns
	HAL_GPIO_WritePin(GPIOF, Row0_Pin|Row1_Pin|Col0_Pin|Col1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, Row2_Pin|Col3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, Row3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, Col2_Pin, GPIO_PIN_SET);

	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_0 | GPIO_OSPEEDER_OSPEEDR15_0
					| GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR15_1;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_0 | GPIO_OSPEEDER_OSPEEDR0_1;
	GPIOG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1;

	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0 | GPIO_OSPEEDER_OSPEEDR13_0
					| GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR13_1;
	GPIOG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14_0 | GPIO_OSPEEDER_OSPEEDR14_1;
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_0 | GPIO_OSPEEDER_OSPEEDR10_1;
}

/**
  * @brief	Sets all columns as output.
  * @param	None
  * @retval None
  */
void Set_Columns_Output()
{
	GPIOF->MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER15_0;
	GPIOE->MODER |= GPIO_MODER_MODER0_0;
	GPIOG->MODER |= GPIO_MODER_MODER8_0;
}

/**
  * @brief	Sets all columns as input.
  * @param	None
  * @retval None
  */
void Set_Columns_Input()
{
	GPIOF->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER15_0);
	GPIOE->MODER &= ~(GPIO_MODER_MODER0_0);
	GPIOG->MODER &= ~(GPIO_MODER_MODER8_0);
}

/**
  * @brief	Sets all rows as output.
  * @param	None
  * @retval None
  */
void Set_Rows_Output()
{
	GPIOF->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
	GPIOG->MODER |= GPIO_MODER_MODER14_0;
	GPIOD->MODER |= GPIO_MODER_MODER10_0;
}

/**
  * @brief	Sets all rows as input.
  * @param	None
  * @retval None
  */
void Set_Rows_Input()
{
	GPIOF->MODER &= ~(GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0);
	GPIOG->MODER &= ~(GPIO_MODER_MODER14_0);
	GPIOD->MODER &= ~(GPIO_MODER_MODER10_0);
}

/**
  * @brief	Writes keypad data to buffer
  * @param	Data
  * @retval	None
  */
void Keypad_Write_Buffer(uint8_t data)
{
	keypad_buffer[keypad_wr_pnt] = data;
	keypad_wr_pnt++;
	keypad_buf_length++;
	if (keypad_wr_pnt == KEYPAD_BUF_SIZE)
	{
	  keypad_wr_pnt = 0;
	}
}

/**
  * @brief	Reads keypad data.
  * @param	None
  * @retval None)
  */
void Read_Keypad()
{
	uint8_t row_pressed_counter = 0;
	uint8_t col_pressed_counter = 0;

	switch(state)
	{
		case IDLE:

			Set_Columns_Output();
			Set_Rows_Input();

			//read rows
			for (int i = 0; i < ROW_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED)
				{
					row_key = pos[i];
					row_pressed_counter++;
				}
			}

			Set_Rows_Output();
			Set_Columns_Input();

			//read columns
			for (int i = 0; i < COL_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED)
				{
					col_key = pos[i];
					col_pressed_counter++;
				}
			}

			//if pressed more than one goes to "Error" state
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY)
			{
				row_key = 0;
				col_key = 0;

				state = ERROR;

			//if pressed single key after debounce time goes to "SINGLE" state
			} else if (col_pressed_counter == SINGLE_KEY && row_pressed_counter == SINGLE_KEY)
			{
				new_pressed_key = (row_key << 4) | (col_key & 0x0F);

				if (new_pressed_key != old_pressed_key && old_pressed_key != 0)
				{
					debounce = 0;
				}
				old_pressed_key = new_pressed_key;

				debounce++;

				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;
					row_key = 0;
					col_key = 0;

					state = SINGLE;
				}
			}

			break;

		case ERROR:

			Set_Columns_Output();
			Set_Rows_Input();

			for (int i = 0; i < ROW_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED)
				{
					row_pressed_counter++;
				}
			}

			Set_Rows_Output();
			Set_Columns_Input();

			for (int i = 0; i < COL_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED)
				{
					col_pressed_counter++;
				}
			}

			//if released all buttons after debounce time goes to "IDLE" state
			if (row_pressed_counter == 0 && col_pressed_counter == 0)
			{
				debounce++;

				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;
					state = IDLE;
				}
			}

			break;

		case SINGLE:

			Set_Columns_Output();
			Set_Rows_Input();

			for (int i = 0; i < ROW_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED)
				{
					row_pressed_counter++;
				}
			}

			Set_Rows_Output();
			Set_Columns_Input();

			for (int i = 0; i < COL_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED)
				{
					col_pressed_counter++;
				}
			}

			//if pressed second key after hold pressing first key goes to "Error" state
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY)
			{
				row_key = 0;
				col_key = 0;
				state = ERROR;

			//if released the pressed key writes char to the buffer
			} else if (col_pressed_counter == 0 && row_pressed_counter == 0)
			{
				debounce++;

				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;

					uint8_t data = Convert_Key_to_Char(new_pressed_key);
					row_key = 0;
					col_key = 0;
					new_pressed_key = 0;
					Keypad_Write_Buffer(data);

					//HAL_UART_Transmit(&huart3, &data, 1, 0xFFFF);
					//HAL_UART_Transmit(&huart3, "\r\n", sizeof("\r\n"), 0xFFFF);

					state = IDLE;
				}
			}

			break;
	}

}
/**
  * @brief	Converts key to char.
  * @param	None
  * @retval return key char or 0(no pressed)
  */
uint8_t Convert_Key_to_Char(uint8_t key)
{
	switch(key)
	{
		case 0x11:
			return '1';
		case 0x12:
			return '2';
		case 0x14:
			return '3';
		case 0x18:
			return 'A';
		case 0x21:
			return '4';
		case 0x22:
			return '5';
		case 0x24:
			return '6';
		case 0x28:
			return 'B';
		case 0x41:
			return '7';
		case 0x42:
			return '8';
		case 0x44:
			return '9';
		case 0x48:
			return 'C';
		case 0x81:
			return '*';
		case 0x82:
			return '0';
		case 0x84:
			return '#';
		case 0x88:
			return 'D';
		default:
			return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
int32_t encoder_value = 0;
/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		 if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
		 {
			 encoder_value--;
		 } else {
			 encoder_value++;
		 }
	}
}

/**
  * @brief	Reads the encoder value to determine
  * 		the real position of the brush.
  * @param	None
  * @retval Can be 0-65535
  */
uint16_t Read_Encoder()
{
	/* Reads encoder value */
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
/**
  * @brief	Saves the brush coordinate to the Backup register.
  * @param	The address of the Backup register
  * @param	The value to be saved
  * @retval None
  */
void Save_Coord(float coord)
{
	/* To write to the Backup register user needs to
	 * Unlock the Backup register to access it
	 * Write value to the Backup Register
	 * Lock the backup register
	 */
	uint16_t l_coord = coord * 10;

	/*set the DBP bit the Power Control
	Register (PWR_CR) to enable access to the Backup
	registers and RTC.*/
	HAL_PWR_EnableBkUpAccess();

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, l_coord);

	HAL_PWR_DisableBkUpAccess();
}

/**
  * @brief	Reads the value of the Backup register.
  * @param	Address of the backup register
  * @retval The real coordinate of the brush
  */
uint16_t Read_Coord()
{

	uint16_t data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

	/* Reads real coordinate of the brush */
	return data;
}

/**
  * @brief	Sets the direction and the speed of the inverter.
  * @param  Direction of the inverter
  * 		To move the brush in the determined direction.
  * 		Can be FORWARD, BACK, STOP.
  * @param	Speed of the inverter
  * 		To move the brush at the determined speed.
  * @retval None
  */
void Set_Inverter(uint8_t dir, uint16_t speed)
{
	/* This function sets the direction
	 * using Brush_Forward, Brush_Back pins
	 * Sets the speed by setting PWM or DAC values
	 */
}

/**
  * @brief	Changes the speed of the inverter.
  * @param	Speed of the inverter
  * 		To move the brush at the determined speed.
  * @retval None
  */
void Change_Speed(uint16_t speed)
{
	/* Changes the speed by changing PWM duty cycle or DAC value */
}

/**
  * @brief	Turns off the brake of the brush to move it.
  * @param	None
  * @retval None
  */
void Brush_Brake_Off()
{
	/* Sets Brush_Brake pin */
}

/**
  * @brief	Turns on the brake of the brush to fix it.
  * @param	None
  * @retval None
  */
void Brush_Brake_On()
{
	/* Sets Brush_Brake pin */
}

/**
  * @brief	Moves the brush in the determined direction.
  * 		and at the specified speed.
  * @param	None
  * @retval None
  */
void Move_Brush()
{

}

/**
  * @brief	Turns on solid1 pin(Pedal_Out) to activate.
  *			Cutting and Pressing contacts.
  * @param	None
  * @retval None
  */
void Solid_On()
{
	/* Sets Pedal_Out pin */
}

/**
  * @brief	Turns off Pedal_Out pin(solid_n1) to deactivate
  *			Cutting and Pressing contacts.
  * @param	None
  * @retval None
  */
void Solid_Off()
{
	/* Sets Pedal_Out pin */
}

/**
  * @brief	Turns on Pressing output to press the paper.
  * @param	None
  * @retval None
  */
void Pressing_On()
{
	/* Sets Pressing pin */
}

/**
  * @brief	Turns off Pressing output to release the paper.
  * @param	None
  * @retval None
  */
void Pressing_Off()
{
	/* Sets Pressing pin */
}

/**
  * @brief	Turns on Cutting output (knife) to cut the paper.
  * @param	None
  * @retval None
  */
void Cutting_On()
{
	/* Sets Cutting pin */
}

/**
  * @brief	Turns off Cutting output (knife).
  * @param	None
  * @retval None
  */
void Cutting_Off(void)
{
	/* Sets Cutting pin */
}

/**
  * @brief	Reads the state of the pedal (pressed or not pressed).
  * @param	None
  * @retval Can be 0/1
  */
uint8_t Read_Pedal(void)
{
	/* Reads Pedal_In pin */
	return 0;
}

/**
  * @brief	Reads the sensors which are attached to the knife
  * 		to determine the position of the knife.
  * @param	None
  * @retval Can be 0-3
  */
uint8_t Read_Knife_Sensors(void)
{
	/* Read the knife sensors pins Knife_Sensor1, Knife_Sensor2*/
	return 0;
}

/**
  * @brief	Reads Hand_Catch input to detect catching of the brush handle.
  * @param	None
  * @retval Can be 0 or 1
  */
uint8_t Read_Hand_Catch_Input()
{
	return 0;
}
