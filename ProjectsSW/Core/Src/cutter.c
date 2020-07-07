/*
 * cutter.c
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "cutter.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;
extern DAC_HandleTypeDef hdac;

///////////////////////////////////////////////////////////////////////////////
//							       LCD									     //
///////////////////////////////////////////////////////////////////////////////
uint8_t lcd_buf_length = 0;
uint8_t lcd_write_pnt = 0;
uint8_t lcd_read_pnt = 0;
uint8_t lcd_timeout = 0;
uint8_t completed = 1;
uint8_t data_arr[4];
uint16_t lcd_ring_buffer[64];

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
	  */
	HAL_Delay(50);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(5);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(1);
	LCD_SendCommand(lcd_addr, 0x30);
	HAL_Delay(10);
	LCD_SendCommand(lcd_addr, 0x02);
	HAL_Delay(10);

	LCD_SendCommand(lcd_addr, 0x28);      // 4 bit mode, 2 line, 5x7 matrix
	LCD_SendCommand(lcd_addr, 0x0C);      // Display on, Cursor off
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
	for (uint8_t i = 0; i < size; ++i)
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

				HAL_I2C_Master_Transmit_IT(&hi2c1, lcd_addr, data_arr, 4);
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
//									KEYPAD									 //
///////////////////////////////////////////////////////////////////////////////
uint8_t keypad_timeout = 0;
uint8_t pos[ROW_SIZE] = {1, 2, 4, 8};
uint8_t keypad_buf_length = 0;
uint8_t keypad_wr_pnt = 0;
uint8_t keypad_rd_pnt = 0;
char 	keypad_buffer[KEYPAD_BUF_SIZE];

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
  * @brief	Reads all rows.
  * @param	Pressed rows counter
  * @retval None
  */
void Read_Rows(uint8_t *row_counter)
{
	Set_Columns_Output();
	Set_Rows_Input();

	for (uint8_t i = 0; i < ROW_SIZE; ++i)
	{
		if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED)
		{
			(*row_counter)++;
		}
	}
}

/**
  * @brief	Reads all columns.
  * @param	Pressed columns counter
  * @retval None
  */
void Read_Columns(uint8_t *col_counter)
{
	Set_Rows_Output();
	Set_Columns_Input();

	for (uint8_t i = 0; i < COL_SIZE; ++i)
	{
		if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED)
		{
			(*col_counter)++;
		}
	}
}

/**
  * @brief	Writes keypad data to buffer
  * @param	Data
  * @retval	None
  */
void Keypad_Write_Buffer(char data)
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
  * @brief	Gets data from the buffer
  * @param	Buffer
  * @retval	Read data
  */
char Read_Keypad_Buffer(char *buffer)
{
	char data = keypad_buffer[keypad_rd_pnt];
	keypad_buf_length--;
	keypad_rd_pnt++;
	if (keypad_rd_pnt == KEYPAD_BUF_SIZE)
	{
		keypad_rd_pnt = 0;
	}
	return data;
}

/**
  * @brief	Converts key to char.
  * @param	None
  * @retval return key char or 0(no pressed)
  */
char Convert_Key_to_Char(uint8_t key)
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

/**
  * @brief	Reads pressed key.
  * @param	None
  * @retval None
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

			//Reads rows
			for (uint8_t i = 0; i < ROW_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED)
				{
					//get pressed row number
					row_key = pos[i];
					row_pressed_counter++;
				}
			}

			Set_Rows_Output();
			Set_Columns_Input();

			//Reads columns
			for (uint8_t i = 0; i < COL_SIZE; ++i)
			{
				if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED)
				{
					//Gets pressed column number
					col_key = pos[i];
					col_pressed_counter++;
				}
			}

			//If pressed more than one keys goes to "Error" state
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY)
			{
				row_key = 0;
				col_key = 0;
				debounce = 0;
				state = ERROR;

			//If pressed single key goes to "SINGLE" state after debounce time
			} else if (col_pressed_counter == SINGLE_KEY && row_pressed_counter == SINGLE_KEY)
			{
				new_pressed_key = (row_key << 4) | (col_key & 0x0F);

				//If pressed key changed resets debounce counter
				if (new_pressed_key != old_pressed_key && old_pressed_key != 0)
				{
					debounce = 0;
				}
				old_pressed_key = new_pressed_key;

				debounce++;

				//After debounce time goes "SINGLE" state
				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;
					row_key = 0;
					col_key = 0;
					state = SINGLE;
				}
			} else {
				debounce = 0;
			}

			break;

		case ERROR:

			Read_Rows(&row_pressed_counter);
			Read_Columns(&col_pressed_counter);

			//If released all buttons after debounce time goes to "IDLE" state
			if (row_pressed_counter == 0 && col_pressed_counter == 0)
			{
				debounce++;

				//After debounce time goes "IDLE" state
				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;
					state = IDLE;
				}
			} else {
				debounce = 0;
			}

			break;

		case SINGLE:

			Read_Rows(&row_pressed_counter);
			Read_Columns(&col_pressed_counter);

			//If pressed second key after hold pressing first key goes to "Error" state
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY)
			{
				row_key = 0;
				col_key = 0;
				state = ERROR;

			//If released the pressed key
			} else if (col_pressed_counter == 0 && row_pressed_counter == 0)
			{
				debounce++;

				//After debounce time writes key to buffer and goes to "IDLE" state
				if (debounce == DEBOUNCE_TIME)
				{
					debounce = 0;

					//Converts pressed key code to character
					char data = Convert_Key_to_Char(new_pressed_key);

					new_pressed_key = 0;

					Keypad_Write_Buffer(data);

					//HAL_UART_Transmit(&huart3, &data, 1, 0xFFFF);
					//HAL_UART_Transmit(&huart3, "\r\n", sizeof("\r\n"), 0xFFFF);

					state = IDLE;
				}
			} else {
				debounce = 0;
			}

			break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//							CHANGE COORDINATE								 //
///////////////////////////////////////////////////////////////////////////////
uint8_t num_pos 			= 0;
uint8_t number_accept_count = 0;
uint8_t mode 				= APPLY_MODE;
uint8_t direction 			= 0;
char coord_array[6] 		= {'0','0','0','0','0','0'};
uint32_t encoder_diff 		= 0;
float set_coord 			= 0;
float initial_coord			= 0;
float coord_diff 			= 0;
uint16_t inverter_speed		= 300;

extern float real_coord;

/**
  * @brief	Writes data to LCD buffer.
  * @param	Data to be written
  * @param  Data size
  * @param  Cursor position
  * @retval None
  */
void Write_LCD_Buffer(char* buf, uint8_t size, uint8_t cursor)
{
	/*for (uint8_t i = 0; i < LCD_BUF_SIZE; ++i)
	{
		lcd_ring_buffer[i] = 0;
	}*/
	//lcd_write_pnt = lcd_read_pnt = 0;
	//lcd_buf_length = 0;

	uint16_t lcd_buf[LCD_BUF_SIZE];
	lcd_buf[0] = cursor;

	for (uint8_t i = 1; i < size; ++i)
	{
		lcd_buf[i] = buf[i] | 0x0100;
	}

	if (buf[5] != '.')
	{
		if (cursor == ROW_2 || cursor == ROW_1)
		{
			uint16_t temp = 0;
			temp = lcd_buf[5];
			lcd_buf[5] = '.' | 0x0100;
			lcd_buf[6] = temp;
		}
	}
	LCD_Write_Buffer(lcd_buf, size);
}

/**
  * @brief	Creates set coordinate value from collected digits.
  * @param	Buffer with digits
  * @retval Set coordinate
  */
float Create_Number(char* buf)
{
	float coord = ((buf[1]-'0')*1000) + ((buf[2]-'0')*100) + ((buf[3]-'0')*10)
			+ (buf[4]-'0') + ((buf[5]-'0')*0.1);

	return coord;
}

void Reset_Pointers()
{
	lcd_write_pnt = lcd_read_pnt = 0;
	lcd_buf_length = 0;
}


void Gets_Direction_and_Diff()
{
	//Gets difference between real and set coordinates
	coord_diff = real_coord - set_coord;
	//Calculates encoder value for coordinate difference
	encoder_diff = (abs(coord_diff)*1000)/12; //1000 value - 12mm
	initial_coord = real_coord;
	//Defines direction
	if (coord_diff < 0)
	{
		direction = FORWARD;
	} else {
		direction = BACK;
	}
	//Unlocks brush to move it
	Brush_Unlock();
	//Starts DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	//Defines initial speed value
	inverter_speed = 300;
	//Sets speed values
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, inverter_speed);
}

void Collects_Digits(char data, int8_t coord_name)
{
	//if gets number
	if (data >= '0' && data <= '9')
	{
		//HAL_UART_Transmit(&huart3, "Number mode\n\r", sizeof("Number mode\n\r"), 0xFFFF);

		if (number_accept_count == 0)
		{
			uint8_t all_empty = 1;

			if (data == '0')
			{
				for (uint8_t i = 1; i < 6; ++i)
				{
					if (coord_array[i] != '0')
					{
						all_empty = 1;
						break;
					} else {
						all_empty = 0;
					}
				}
			}

			if (all_empty == 1)
			{
				//HAL_UART_Transmit(&huart3, "Go to EDIT mode\n\r", sizeof("Go to EDIT mode\n\r"), 0xFFFF);

				if (num_pos < 5)
				{
					num_pos++;

					for (uint8_t i = 1; i < 6; ++i)
					{
						coord_array[i] = coord_array[i+1];
					}
					coord_array[5] = data;

					Reset_Pointers();
					if (coord_name == REAL)
					{
						Write_LCD_Buffer(coord_array, 7, ROW_1);
					} else {
						Write_LCD_Buffer(coord_array, 7, ROW_2);
					}
				}
			}
		}

	//if gets * to delete digit
	} else if (data == '*') {

		if (number_accept_count == 0)
		{
			if (num_pos > 0)
			{
				num_pos--;

				for (uint8_t i = 1; i < 6; ++i)
				{
					coord_array[6-i] = coord_array[6-i-1];
				}
				coord_array[1] = '0';

				Reset_Pointers();
				if (coord_name == REAL)
				{
					Write_LCD_Buffer(coord_array, 7, ROW_1);
				} else {
					Write_LCD_Buffer(coord_array, 7, ROW_2);
				}
			}
		} else if (number_accept_count == 1)
		{
			number_accept_count = 0;
			Reset_Pointers();
			Write_LCD_Buffer((char*)"      Edit Mode      ", LCD_ROW_SIZE, ROW_4);
		}
	}

	if (data == '#')
	{
		number_accept_count++;

		if (number_accept_count == 1)
		{
			//Creates number from collected digits
			if (coord_name == SET)
			{
				set_coord = Create_Number(coord_array);
			} else {
				real_coord = Create_Number(coord_array);
			}
			Reset_Pointers();
			Write_LCD_Buffer((char*)"    Are you sure?    ", LCD_ROW_SIZE, ROW_4);

		} else if (number_accept_count == 2)
		{
			number_accept_count = 0;
			//After pressing # of second time goes BRUSH_MOVE mode
			Reset_Pointers();
			if (coord_name == REAL)
			{
				Print_Coord(set_coord, SET);
				Write_LCD_Buffer((char*)"                     ", LCD_ROW_SIZE, ROW_3);
				Write_LCD_Buffer((char*)"    Are you sure?    ", LCD_ROW_SIZE, ROW_4);
				Save_Coord(real_coord);
				mode = APPLY_MODE;
			} else {
				Write_LCD_Buffer((char*)"  Brush Moving Mode  ", LCD_ROW_SIZE, ROW_4);
				Gets_Direction_and_Diff();
				//Passes to Brush Move mode
				mode = BRUSH_MOVE;
			}
		}
	}

}

/**
  * @brief	Checks pressed key.
  * @param	None
  * @retval None
  */
void Check_Pressed_Key()
{
	char data = 0;

	if (!Empty(keypad_buf_length))
	{
		data = Read_Keypad_Buffer(keypad_buffer);

		//If pressed 'C' key goes to the CALLIBRATION mode
		if (data == 'C')
		{
			Reset_Pointers();
			Write_LCD_Buffer((char*)"                     ", LCD_ROW_SIZE, 0xC0);
			Write_LCD_Buffer((char*)"     Callibration    ", LCD_ROW_SIZE, 0x94);
			Write_LCD_Buffer((char*)"                     ", LCD_ROW_SIZE, 0xD4);
			memset(coord_array, '0', 6);
			mode = CALLIBRATION;
		}

		if (mode == APPLY_MODE)
		{
			if (data == '*')
			{
				Reset_Pointers();
				Write_LCD_Buffer((char*)" 000000", 7, ROW_2);
				Write_LCD_Buffer((char*)"      Edit Mode      ", LCD_ROW_SIZE, ROW_4);
				//Goes to edit mode
				mode = EDIT;
				memset(coord_array, '0', 6);
				//HAL_UART_Transmit(&huart3, "Go to EDIT mode\n\r", sizeof("Go to EDIT mode\n\r"), 0xFFFF);

			} else if (data == '#')
			{
				Reset_Pointers();
				Write_LCD_Buffer((char*)"                     ", LCD_ROW_SIZE, ROW_4);
				mode = CHECK_PEDAL;
			}
		} else if (mode == EDIT)
		{
			Collects_Digits(data, SET);

		} else if (mode == CALLIBRATION)
		{
			Collects_Digits(data, REAL);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//					Brush Moving Mode                                        //
///////////////////////////////////////////////////////////////////////////////
int32_t encoder_value = 0;
uint16_t speed = 0;

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
	if (dir == FORWARD)
	{
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port,	Brush_Back_Pin, GPIO_PIN_SET);
	} else if (dir == BACK) {
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port, Brush_Back_Pin, GPIO_PIN_RESET);
	} else if (dir == STOP) {
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port, Brush_Back_Pin, GPIO_PIN_SET);
	}

	//Sets DAC value
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
}

/**
  * @brief	Changes the speed of the inverter.
  * @param	Speed of the inverter
  * 		To move the brush at the determined speed.
  * @retval None
  */
void Change_Speed(uint16_t *speed)
{
	if (*speed <= MAX_DAC_VALUE)
	{
		*speed = *speed + RAMP_UP;

		if (*speed > MAX_DAC_VALUE)
		{
			*speed = MAX_DAC_VALUE;
		}
	}
	if (speed >= 0)
	{
		*speed = *speed - RAMP_DOWN;
		if (*speed < 0)
		{
			*speed = 0;
		}
	}
	/* Changes the speed by changing PWM duty cycle or DAC value */
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, *speed);
}

/**
  * @brief	Unlocks the brush to move it.
  * @param	None
  * @retval None
  */
void Brush_Unlock()
{
	 HAL_GPIO_WritePin(Brush_Lock_GPIO_Port, Brush_Lock_Pin, GPIO_PIN_RESET);
}

/**
  * @brief	Locks the brush to fix it.
  * @param	None
  * @retval None
  */
void Brush_Lock()
{
	HAL_GPIO_WritePin(Brush_Lock_GPIO_Port, Brush_Lock_Pin, GPIO_PIN_SET);
}

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
  * @brief	Moves the brush in the determined direction.
  * 		and at the specified speed.
  * @param	None
  * @retval None
  */
void Move_Brush()
{
	if (mode == BRUSH_MOVE)
	{
		//if board is powered and brush is unlocked
		//if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 1 &&
		//		HAL_GPIO_ReadPin(Brush_Lock_GPIO_Port, Brush_Lock_Pin) == 0)
		//{

		if (real_coord > set_coord)
		{
			real_coord = initial_coord - ((float)(12*abs(encoder_value))/1000);
		} else {
			real_coord = initial_coord + ((float)(12*abs(encoder_value))/1000);
		}

		if ((direction == FORWARD && real_coord > LIMIT_UP) || (direction == BACK && real_coord < LIMIT_DOWN))
		{
			Set_Inverter(STOP, speed);
			Brush_Lock();
			Save_Coord(real_coord);
			mode = CHECK_PEDAL;

		} else {

			if ((encoder_diff - abs(encoder_value)) > COORD_DIFF)
			{
				Change_Speed(&speed);
				Set_Inverter(direction, speed);

			} else if (((encoder_diff - abs(encoder_value)) <= COORD_DIFF) &&
					((encoder_diff - abs(encoder_value)) > 5))
			{
				Change_Speed(&speed);
				Set_Inverter(direction, speed);

			} else
			{
				//Turns off inverter
				Set_Inverter(STOP, speed);
				//Locks brush to fix it
				Brush_Lock();
				//Saves real coordinate to backup register
				Save_Coord(real_coord);
				//To do write to LCD real coordinate

				if (real_coord > set_coord)
				{
					real_coord = initial_coord - ((float)(12*abs(encoder_value))/1000);
				} else {
					real_coord = initial_coord + ((float)(12*abs(encoder_value))/1000);
				}

				encoder_value = 0;
				Print_Coord(real_coord, REAL);
				Write_LCD_Buffer((char*)"                     ", LCD_ROW_SIZE, ROW_4);

				//Passes to CHECK_PEDAL mode
				mode = CHECK_PEDAL;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//					     CUTTING MODE										 //
///////////////////////////////////////////////////////////////////////////////
Input_State input_state;
uint8_t cut_is_done = 0;
uint16_t delay_for_cutting_buttons = 0;
uint16_t delay_for_cutting = 0;

/**
  * @brief	Reads specified inputs.
  * @param	None
  * @retval None
  */
void Read_Inputs(void)
{
	Read_Pin(Cutting_Buttons_GPIO_Port, Cutting_Buttons_Pin, &input_state.cut_cnt_for_st0,
				 &input_state.cut_cnt_for_st1, &input_state.cut_is_pressed, 0);

	Read_Pin(Pedal_In_GPIO_Port, Pedal_In_Pin, &input_state.pedal_cnt_for_st0,
			 &input_state.pedal_cnt_for_st1, &input_state.pedal_is_pressed, 1);

	Read_Pin(Hand_Catch_GPIO_Port, Hand_Catch_Pin, &input_state.hand_catch_cnt_for_st0,
 &input_state.hand_catch_cnt_for_st1,	&input_state.hand_catch_is_pressed, 1);
}

/**
  * @brief	Reads the specified input state.
  * @param	Port of specified GPIO
  * @param	Pin of specified GPIO
  * @param	Pointer to save counter of 0 state
  * @param	Pointer to save counter of 1 state
  * param	Pointer to save pressed(1) or no pressed(0) state
  * param	State by which pins is turned on. 0 or 1
  * @retval None
  */
void Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t * st0_counter,
				uint8_t * st1_counter, uint8_t * is_pressed, uint8_t on_state)
{
	uint8_t pin_data = 0xFF;

	//read desired button pin
	pin_data = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	//if button is pressed
	if (pin_data == on_state)
	{
		//reset counter for state1
		*st1_counter = 0;

		// Increment State0 counter
		if (*st0_counter < DEBOUNCE_TIME+1)
		{
			(*st0_counter)++;
		}
		//if state0 counter is equal to DEBOUNCE_TIME, we consider that button is pressed.
		if (*st0_counter == DEBOUNCE_TIME)
		{
			*is_pressed = 1;
		}
	//if button not pressed
	} else {

		*st0_counter = 0;

		if (*st1_counter < DEBOUNCE_TIME+1)
		{
			(*st1_counter)++;
		}
		if (*st1_counter == DEBOUNCE_TIME)
		{
			if (GPIOx == Cutting_Buttons_GPIO_Port && GPIO_Pin == Cutting_Buttons_Pin)
			{
				cut_is_done = 0;
			}
			*is_pressed = 0;
		}
	}
}

/**
  * @brief	Activates cutting buttons.
  * @param	None
  * @retval None
  */
void Cutting_Button_On()
{
	HAL_GPIO_WritePin(Cutting_Buttons_Allow_GPIO_Port, Cutting_Buttons_Allow_Pin, GPIO_PIN_RESET);
}

/**
  * @brief	Deactivate cutting buttons.
  * @param	None
  * @retval None
  */
void Cutting_Button_Off()
{
	HAL_GPIO_WritePin(Cutting_Buttons_Allow_GPIO_Port, Cutting_Buttons_Allow_Pin, GPIO_PIN_SET);
}

/**
  * @brief	Turns on Cutting output (knife) to cut the paper.
  * @param	None
  * @retval None
  */
void Cutting_On()
{
	HAL_GPIO_WritePin(Cutting_GPIO_Port, Cutting_Pin, GPIO_PIN_RESET);
}

/**
  * @brief	Turns off Cutting output (knife).
  * @param	None
  * @retval None
  */
void Cutting_Off(void)
{
	HAL_GPIO_WritePin(Cutting_GPIO_Port, Cutting_Pin, GPIO_PIN_SET);
}

/**
  * @brief	Reads the sensors which are attached to the knife
  * 		to determine the position of the knife.
  * @param	None
  * @retval Can 0,1
  */
uint8_t Read_Knife_Sensors(void)
{
	if (HAL_GPIO_ReadPin(Knife_Sensor1_GPIO_Port, Knife_Sensor1_Pin) == 1)
	{
		/* Read the knife sensors pins Knife_Sensor1, Knife_Sensor2*/
		return 1;
	}
	return 0;
}

uint8_t pedal_is_pressed = 0;
uint8_t temp = 0;
uint8_t old_temp = 0;

void Check_Pedal()
{
	//if pedal is pressed
	if (input_state.pedal_is_pressed == 1)
	{
		pedal_is_pressed = 1;

		//if passed 5 second
		if (delay_for_cutting_buttons == 5000)
		{
			//Activates cuttings button
			Cutting_Button_On();

			//if cutting buttons is pressed
			if ((input_state.cut_is_pressed == 1) && (cut_is_done == 0))
			{
				//if passed 3 second
				if (delay_for_cutting == 3000)
				{
					//Reads knife sensors
					if (Read_Knife_Sensors() == 1)
					{
						//Cuts the paper
						Cutting_On();

						temp = 1;
						if (old_temp != temp)
						{
							Reset_Pointers();
							Write_LCD_Buffer((char*)"       Cutting       ", LCD_ROW_SIZE, ROW_4);
						}
					} else {

						//Deactivates cutting buttons
						Cutting_Button_Off();
						Cutting_Off();
						cut_is_done = 1;

						temp = 2;
						if (old_temp != temp)
						{
							Reset_Pointers();
							Write_LCD_Buffer((char*)"     Cut is done     ", LCD_ROW_SIZE, ROW_4);
						}

					}
				}
			} else if ((input_state.cut_is_pressed == 0) && (cut_is_done == 0) ) {
				delay_for_cutting = 0;

				temp = 3;
				if (old_temp != temp)
				{
					Reset_Pointers();
					Write_LCD_Buffer((char*)"     Cutting Mode    ", LCD_ROW_SIZE, ROW_4);
				}
			}
		}
	} else {
		if (pedal_is_pressed == 1)
		{
			old_temp = temp = 0;
			pedal_is_pressed = 0;
			delay_for_cutting_buttons = 0;
			delay_for_cutting = 0;
			Cutting_Button_Off();
			Reset_Pointers();
			Write_LCD_Buffer((char*)"    Are you sure?    ", LCD_ROW_SIZE, ROW_4);
			num_pos = 0;
			memset(coord_array, '0', 6);
			mode = APPLY_MODE;
			//HAL_UART_Transmit(&huart3, "Pedal released\n\r", sizeof("Pedal released\n\r"), 0xFFFF);
		}
	}
	old_temp = temp;
}

///////////////////////////////////////////////////////////////////////////////
//							HAND CATCH										 //
///////////////////////////////////////////////////////////////////////////////
uint8_t hand_catch_detected = 0;

void Print_Coord(float r_coord, uint8_t coord_name)
{
	char temp_buf[10];
	sprintf(temp_buf+1, "%6.1f", r_coord);
	for (int i = 0; i < strlen(temp_buf); ++i)
	{
	  if (temp_buf[i] == 0x20)
	  {
		  temp_buf[i] = '0';
	  }

	}
	Reset_Pointers();
	if (coord_name == REAL)
	{
		Write_LCD_Buffer((char*)" Real  ", sizeof(" Real  "), 0x80);
		Write_LCD_Buffer(temp_buf, 7, 0x86);
	} else {
		Write_LCD_Buffer((char*)" Set   ", sizeof(" Set   "), 0xC0);
		Write_LCD_Buffer(temp_buf, 7, 0xC6);
	}
}

void Check_Hand_Catch()
{
	if (input_state.hand_catch_is_pressed == 1)
	{
		hand_catch_detected = 1;
	} else
	{
		if (hand_catch_detected == 1)
		{
			hand_catch_detected = 0;

			float temp_value = ((float)abs(encoder_value)*12)/1000;

			if (encoder_value < 0)
			{
				real_coord = real_coord - temp_value;
			} else
			{
				real_coord = real_coord + temp_value;
			}

			//Prints real coordinate to LCD
			Print_Coord(real_coord, REAL);
			//Saves real coord to backup register
			Save_Coord(real_coord);
			//Resets encoder value
			encoder_value = 0;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//								Main Task									 //
///////////////////////////////////////////////////////////////////////////////
void Main_Task()
{
	  if ((mode == APPLY_MODE) || (mode == EDIT) || (mode == CALLIBRATION))
	  {
		  Check_Pressed_Key();
	  } else if (mode == BRUSH_MOVE)
	  {
		  Move_Brush();
	  } else if (mode == CHECK_PEDAL)
	  {
		  Check_Pedal();
	  }

	  Check_Hand_Catch();
	  LCD_Write(LCD_ADDR);

	  /*if (encoder_time == 100)
	  {
		  	  char hex[10];

		  	  sprintf(hex, "%d\n\r", encoder_value);
		  	  HAL_UART_Transmit(&huart3, hex, 10, 0xFFFF);
	  }*/

	  if (keypad_timeout == KEYPAD_TIMEOUT)
	  {
		  keypad_timeout = 0;
		  Read_Keypad();
		  Read_Inputs();
	  }
}
