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
#include "math.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;
extern DAC_HandleTypeDef hdac;

uint8_t lcd_buf_length = 0;
uint8_t lcd_write_pnt = 0;
uint8_t lcd_read_pnt = 0;
uint8_t lcd_timeout = 0;
uint8_t completed = 1;
uint8_t data_arr[4];
uint16_t lcd_ring_buffer[LCD_BUF_SIZE];

uint8_t keypad_timeout = 0;
uint8_t input_timeout = 0;
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

uint8_t coord_size 			= 0;
uint8_t number_accept_count = 0;
uint8_t mode 				= SELECT;
uint8_t direction 			= 0;
uint8_t is_min_speed			= 0;
char coord_array[COORD_SIZE];
double encoder_diff 		= 0;
double set_coord 			= 0;
double initial_coord			= 0;
double coord_diff 			= 0;
int32_t encoder_value 		= 0;
int32_t previous_encoder_value 		= 0;
extern double real_coord;

uint16_t speed = MIN_SPEED;

uint8_t arrange_out = 0;
uint16_t timeout_for_ramp = 0;
uint8_t is_move = 0;
extern uint8_t time_for_change_ramp;

Input_State input_state;
uint8_t cut_is_done = 0;
uint16_t delay_for_cutting_buttons = 0;
uint16_t delay_for_cutting = 0;

uint8_t pedal_is_pressed = 0;
uint8_t temp = 0;
uint8_t old_temp = 0;

uint8_t hand_catch_detected = 0;

uint16_t print_real_coord_time = 0;

uint8_t is_limited_number = 0;

double temp_coord = 0;

extern osMessageQueueId_t myQueue01Handle;
///////////////////////////////////////////////////////////////////////////////
//							       LCD									     //
///////////////////////////////////////////////////////////////////////////////
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
	HAL_Delay(150); // / 50 er
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
  if (buff_size == LCD_BUF_SIZE) {
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
  if (buff_size == 0) {
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
	for (uint8_t i = 0; i < size; ++i) {
	  if (!Full(lcd_buf_length)) {
		  lcd_ring_buffer[lcd_write_pnt] = data[i];
		  lcd_write_pnt++;
		  lcd_buf_length++;
		  if (lcd_write_pnt == LCD_BUF_SIZE) {
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

  if (lcd_read_pnt == LCD_BUF_SIZE) {
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
	if (!Empty(lcd_buf_length)) {
		uint16_t data = LCD_Read_Buffer();
		uint8_t up = data & 0xF0;
		uint8_t lo = (data << 4) & 0xF0;
		uint8_t rs = 0;
		rs = PIN_RS;

		//if data
		if ((data & LCD_DATA_MASK) == LCD_DATA_MASK) {
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

		if (completed == 1) {
			completed = 0;
			HAL_I2C_Master_Transmit_IT(&hi2c1, lcd_addr, data_arr, 4);
		}
	}
}

/**
  * @brief	Displays a char on the LCD with non-blocking mode.
  * @retval None
  */
void LCD_Write1(uint8_t lcd_addr, uint16_t data)
{
	//every ms
	//if (!Empty(lcd_buf_length)) {
		//uint16_t data = LCD_Read_Buffer();
		uint8_t up = data & 0xF0;
		uint8_t lo = (data << 4) & 0xF0;
		uint8_t rs = 0;
		rs = PIN_RS;

		//if data
		if ((data & LCD_DATA_MASK) == LCD_DATA_MASK) {
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

		//if (completed == 1) {
		//	completed = 0;
			HAL_I2C_Master_Transmit_IT(&hi2c1, lcd_addr, data_arr, 4);
		//}
	//}
}

void LCD_Init1(uint8_t lcd_addr)
{
	/* Initializes the following
	* Function Set
	* Entry mode
	* Display on/off
	* Clear Display
	  */
	HAL_Delay(150); // / 50 er
	LCD_Write1(LCD_ADDR, 0x30);
	HAL_Delay(50);
	HAL_Delay(5);
	LCD_Write1(LCD_ADDR, 0x30);
	HAL_Delay(50);
	HAL_Delay(1);
	LCD_Write1(LCD_ADDR, 0x30);
	HAL_Delay(50);
	HAL_Delay(10);
	LCD_Write1(LCD_ADDR, 0x02);
	HAL_Delay(50);
	HAL_Delay(10);

	LCD_Write1(LCD_ADDR, 0x28);      // 4 bit mode, 2 line, 5x7 matrix
	HAL_Delay(50);
	LCD_Write1(LCD_ADDR, 0x0C);      // Display on, Cursor off
	HAL_Delay(50);
	LCD_Write1(LCD_ADDR, 0x01);      // Clear Display Screen
	HAL_Delay(50);
	LCD_Write1(LCD_ADDR, 0x06);      // Increment cursor, no shift
	HAL_Delay(50);
	LCD_Write1(LCD_ADDR, 0x80);
	HAL_Delay(50);
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

	for (uint8_t i = 0; i < ROW_SIZE; ++i) {
		if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED) {
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

	for (uint8_t i = 0; i < COL_SIZE; ++i) {
		if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED) {
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
	if (keypad_wr_pnt == KEYPAD_BUF_SIZE) {
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
	if (keypad_rd_pnt == KEYPAD_BUF_SIZE) {
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
	switch(key) {
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

	switch(state) {
		case IDLE:

			Set_Columns_Output();
			Set_Rows_Input();

			//Reads rows
			for (uint8_t i = 0; i < ROW_SIZE; ++i) {
				if (HAL_GPIO_ReadPin(row_gpio_port[i], row_gpio_pin[i]) == PRESSED) {
					//get pressed row number
					row_key = pos[i];
					row_pressed_counter++;
				}
			}

			Set_Rows_Output();
			Set_Columns_Input();

			//Reads columns
			for (uint8_t i = 0; i < COL_SIZE; ++i) {
				if (HAL_GPIO_ReadPin(col_gpio_port[i], col_gpio_pin[i]) == PRESSED) {
					//Gets pressed column number
					col_key = pos[i];
					col_pressed_counter++;
				}
			}

			//If pressed more than one keys goes to "Error" state
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY) {
				row_key = 0;
				col_key = 0;
				debounce = 0;
				state = ERROR;

			//If pressed single key goes to "SINGLE" state after debounce time
			} else if (col_pressed_counter == SINGLE_KEY && row_pressed_counter == SINGLE_KEY) {
				new_pressed_key = (row_key << 4) | (col_key & 0x0F);

				//If pressed key changed resets debounce counter
				if (new_pressed_key != old_pressed_key && old_pressed_key != 0) {
					debounce = 0;
				}
				old_pressed_key = new_pressed_key;

				debounce++;

				//After debounce time goes "SINGLE" state
				if (debounce == DEBOUNCE_TIME) {
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
			if (col_pressed_counter > SINGLE_KEY || row_pressed_counter > SINGLE_KEY) {
				row_key = 0;
				col_key = 0;
				state = ERROR;

			//If released the pressed key
			} else if (col_pressed_counter == 0 && row_pressed_counter == 0) {
				debounce++;

				//After debounce time writes key to buffer and goes to "IDLE" state
				if (debounce == DEBOUNCE_TIME) {
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
void Lock_Handle()
{
	HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_RESET);// set er darav reset
}

void Unlock_Handle()
{
	HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_SET); // reset er darav set
}

/**
  * @brief	Writes data to LCD buffer.
  * @param	Data to be written
  * @param  Data size
  * @param  Cursor position
  * @retval None
  */
/*void Write_LCD_Buffer(char* buf, uint8_t size, uint8_t cursor)
{
	uint16_t lcd_buf[LCD_BUF_SIZE];
	lcd_buf[0] = cursor;
	uint8_t l_size = size;

	for (uint8_t i = 1; i < size+1; ++i) {
		lcd_buf[i] = buf[i-1] | LCD_DATA_MASK;
	}
	l_size++;

	if (buf[4] != '.') {
		if (cursor == S_COORD_POS || cursor == R_COORD_POS) {
			uint16_t temp = 0;
			temp = lcd_buf[5];
			lcd_buf[5] = '.' | LCD_DATA_MASK;
			lcd_buf[6] = temp;
			l_size++;
		}
	}
	LCD_Write_Buffer(lcd_buf, l_size);
}*/

/**
  * @brief	Writes data to LCD buffer.
  * @param	Data to be written
  * @param  Data size
  * @param  Cursor position
  * @retval None
  */
void Write_LCD_Buffer(char* buf, uint8_t size, uint8_t cursor)
{
	uint8_t lcd_buf[30];

	if (cursor != ROW_3)
	{
		for (uint8_t i = 0; i < size; ++i) {
			lcd_buf[i] = buf[i];
		}

		if (buf[4] != '.') {
			if (cursor == S_COORD_POS || cursor == R_COORD_POS) {
				uint16_t temp = 0;
				temp = lcd_buf[4];
				lcd_buf[4] = '.';
				lcd_buf[5] = temp;
				lcd_buf[6] = '\0';
			}
		}

		LCD_SendString(LCD_ADDR, (char*)lcd_buf);
	} else {
		LCD_SendString(LCD_ADDR, buf);
	}
}

/**
  * @brief	Creates set coordinate value from collected digits.
  * @param	Buffer with digits
  * @retval Set coordinate
  */
double Create_Number(char* buf)
{
	double coord = ((buf[0]-'0')*1000) + ((buf[1]-'0')*100) + ((buf[2]-'0')*10)
			+ (buf[3]-'0') + ((buf[4]-'0')*0.1);

	return coord;
}

void Reset_LCD_Pointers()
{
	lcd_write_pnt = lcd_read_pnt = 0;
	lcd_buf_length = 0;
}

uint32_t set_tick = 0;

uint8_t Get_Direction_and_Diff()
{
	set_tick = (double)set_coord * ONE_ROTATION_TICK / ONE_ROTATION_VAL;

	//set_tick = set_tick + 30;

	//if ((encoder_value > HARD_LIMIT_UP_IN_TICK) || (encoder_value < LIMIT_DOWN_IN_TICK)) {
	//	return 1;
	//}

	//if ((set_tick > HARD_LIMIT_UP_IN_TICK) || (set_tick < LIMIT_DOWN_IN_TICK)) {
	//	return 1;
	//}

	if (set_tick == encoder_value)
	{
		return 1;
	}

	int32_t tick_diff = encoder_value - set_tick;

	if (abs(tick_diff) < 5)
	{
		return 1;
	}

	/*//Gets difference between real and set coordinates
	coord_diff = real_coord - set_coord;
	double _round = fabs(coord_diff);
	//Calculates encoder value for coordinate difference
	encoder_diff = roundf((double)(_round*ONE_ROTATION_TICK)/ONE_ROTATION_VAL); //1000 value - 12mm
	initial_coord = real_coord;*/

	//Defines direction
	if (tick_diff < 0) {
		direction = FORWARD;
	} else {
		direction = BACK;
	}

	if (abs(tick_diff) < MIN_DISTANCE_IN_TICK) {
		speed = MIN_SPEED;
		is_min_speed = 1;
	} else {
		speed = 0;
		is_min_speed = 0;
	}
	return 0;
}

uint8_t debug = 0;
uint8_t pressed_count = 0;
uint8_t queue_var = 0;

void Collects_Digits(int8_t coord_name)
{
	char data = 0;

	if (!Empty(keypad_buf_length)) {
		data = Read_Keypad_Buffer(keypad_buffer);
	}

	if (data == 'B')
	{
		if  (pressed_count == 0)
		{
			//Reset_LCD_Pointers();
			if (coord_name == SET)
			{
				set_coord = temp_coord;
				coord_size = Get_Coord_Size(coord_array, set_coord);
				//Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);
				//send queue 1
				queue_var = SET_COORD_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);

			} else if (coord_name == REAL)
			{
				real_coord = temp_coord;
				coord_size = Get_Coord_Size(coord_array, real_coord);
				//Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);
				queue_var = REAL_COORD_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);

				//Write_LCD_Buffer((char*)"Set   ", sizeof("Set   "), ROW_2);
				queue_var = SET_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);

				coord_size = Get_Coord_Size(coord_array, set_coord);
				queue_var = SET_COORD_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				//Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);
				//send queue 2
			}
			queue_var = SPACE_3_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_3);
			queue_var = MAIN_MENU_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//Write_LCD_Buffer((char*) " *-Edit #-Cut C-Cal ", LCD_ROW_SIZE, ROW_4);
			mode = SELECT;
		}
	}

	//if gets number
	if (data >= '0' && data <= '9') {
		pressed_count++;
		if (number_accept_count == 0) {
			uint8_t all_zero = 0;
			if (data == '0') {
				for (uint8_t i = 0; i < COORD_SIZE; ++i) {
					if (coord_array[i] != '0') {
						all_zero = 0;
						break;
					} else {
						all_zero = 1;
					}
				}
			}

			if (all_zero == 0) {
				if (coord_size < COORD_SIZE) {
					coord_size++;
					for (uint8_t i = 0; i < COORD_SIZE; ++i) {
						coord_array[i] = coord_array[i+1];
					}
					coord_array[COORD_SIZE-1] = data;

					//Reset_LCD_Pointers();
					if (coord_name == REAL) {
						//Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);
						queue_var = REAL_COORD_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 3
					} else if (coord_name == SET) {
						//Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);
						queue_var = SET_COORD_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 4
					}

					if (is_limited_number == 1)
					{
						is_limited_number = 0;
						if (coord_name == REAL) {
							//Write_LCD_Buffer((char*)"   ", 3, 0x8E);
							queue_var = SPACE_FOR_MAX_MIN_1_ROW_CMD;
							xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
							//send queue 5
						} else {
							//Write_LCD_Buffer((char*)"   ", 3, 0xCE);
							queue_var = SPACE_FOR_MAX_MIN_2_ROW_CMD;
							xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
							//send queue 6
						}
					}
				}
			}
		}

	//if gets * to delete digit
	} else if (data == '*') {
		pressed_count++;
		if (number_accept_count == 0) {
			if (coord_size > 0) {
				coord_size--;

				for (uint8_t i = 0; i < COORD_SIZE-1; ++i) {
					coord_array[COORD_SIZE-i-1] = coord_array[COORD_SIZE-i-2];
				}
				coord_array[0] = '0';

				//Reset_LCD_Pointers();
				if (coord_name == REAL) {
					//Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);
					queue_var = REAL_COORD_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 7
				} else if (coord_name == SET) {
					//Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);
					queue_var = SET_COORD_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 8
				}
				if (is_limited_number == 1)
				{
					is_limited_number = 0;
					if (coord_name == REAL) {
						//Write_LCD_Buffer((char*)"   ", 3, 0x8E);
						queue_var = SPACE_FOR_MAX_MIN_1_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 9
					} else {
						//Write_LCD_Buffer((char*)"   ", 3, 0xCE);
						queue_var = SPACE_FOR_MAX_MIN_2_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 10
					}
				}
			}
		} else if (number_accept_count == 1) {
			number_accept_count = 0;
			//Reset_LCD_Pointers();
			if (coord_name == SET) {
				queue_var = EDIT_MODE_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				//Write_LCD_Buffer((char*)"     Edit Mode      ", LCD_ROW_SIZE, ROW_4);
				//send queue 11
			} else {
				queue_var = SPACE_4_ROW_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
				//send queue 12
			}
		}
	}

	if (data == '#') {
		pressed_count++;
		number_accept_count++;
		if (number_accept_count == 1) {
			//Reset_LCD_Pointers();
			//Creates number from collected digits
			if (coord_name == SET) {
				set_coord = Create_Number(coord_array);
				if (set_coord < LIMIT_DOWN || set_coord > SOFT_LIMIT_UP) {
					number_accept_count = 0;
					char temp_buf[10];
					if (set_coord < LIMIT_DOWN) {
						sprintf(temp_buf, "%6.1f", (double)LIMIT_DOWN);
					} else if (set_coord > SOFT_LIMIT_UP) {
						sprintf(temp_buf, "%6.1f", (double)SOFT_LIMIT_UP);
					}
					for (int i = 0; i < sizeof(temp_buf); ++i) {
					  if (temp_buf[i] == 0x20) {
						  temp_buf[i] = '0';
					  }
					}
					coord_array[0] = temp_buf[0];
					coord_array[1] = temp_buf[1];
					coord_array[2] = temp_buf[2];
					coord_array[3] = temp_buf[3];
					coord_array[4] = temp_buf[5];

					if (set_coord < LIMIT_DOWN) {
						//Write_LCD_Buffer((char*)"Min", 3, 0xCE);
						queue_var = MIN_2_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 13
						set_coord = LIMIT_DOWN;
					} else if (set_coord > SOFT_LIMIT_UP) {
						//Write_LCD_Buffer((char*)"Max", 3, 0xCE);
						queue_var = MAX_2_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 14
						set_coord = SOFT_LIMIT_UP;
					}
					is_limited_number = 1;
					char temp_array[7];
					coord_size = Get_Coord_Size(temp_array, set_coord);
					//Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);
					queue_var = SET_COORD_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 15

				} else {
					queue_var = ARE_YOU_SURE_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					queue_var = SPACE_FOR_MAX_MIN_2_ROW_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//Write_LCD_Buffer((char*)"   Are you sure?    ", LCD_ROW_SIZE, ROW_4);
					//Write_LCD_Buffer((char*)"   ", 3, 0xCE);
					//send queue 16
				}
			} else if (coord_name == REAL) {
				real_coord = Create_Number(coord_array);
				if (real_coord < LIMIT_DOWN || real_coord > SOFT_LIMIT_UP) {
					number_accept_count = 0;
					char temp_buf[10];
					if (real_coord < LIMIT_DOWN) {
						sprintf(temp_buf, "%6.1f", (double)LIMIT_DOWN);
					} else if (real_coord > SOFT_LIMIT_UP) {
						sprintf(temp_buf, "%6.1f", (double)SOFT_LIMIT_UP);
					}
					for (int i = 0; i < sizeof(temp_buf); ++i) {
					  if (temp_buf[i] == 0x20) {
						  temp_buf[i] = '0';
					  }
					}
					coord_array[0] = temp_buf[0];
					coord_array[1] = temp_buf[1];
					coord_array[2] = temp_buf[2];
					coord_array[3] = temp_buf[3];
					coord_array[4] = temp_buf[5];

					if (real_coord < LIMIT_DOWN) {
						//Write_LCD_Buffer((char*)"Min", 3, 0x8E);
						queue_var = MIN_1_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 17
						real_coord = LIMIT_DOWN;
					} else if (real_coord > SOFT_LIMIT_UP) {
						//Write_LCD_Buffer((char*)"Max", 3, 0x8E);
						queue_var = MAX_1_ROW_CMD;
						xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
						//send queue 18
						real_coord = SOFT_LIMIT_UP;
					}
					is_limited_number = 1;
					char temp_array[7];
					coord_size = Get_Coord_Size(temp_array, real_coord);
					//Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);
					queue_var = REAL_COORD_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 19

				} else {
					//Write_LCD_Buffer((char*)"   Are you sure?    ", LCD_ROW_SIZE, ROW_4);
					queue_var = ARE_YOU_SURE_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//Write_LCD_Buffer((char*)"   ", 3, 0x8E);
					queue_var = SPACE_FOR_MAX_MIN_1_ROW_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 20
				}
			}


		} else if (number_accept_count == 2) {
			number_accept_count = 0;

			//Reset_LCD_Pointers();
			if (coord_name == REAL) {
				Print_Coord(set_coord, SET);
				//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_3);
				//Write_LCD_Buffer((char*) " *-Edit #-Cut C-Cal ", LCD_ROW_SIZE, ROW_4);
				queue_var = SPACE_3_ROW_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				queue_var = MAIN_MENU_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				//send queue 21
				encoder_value = (double)real_coord * ONE_ROTATION_TICK / ONE_ROTATION_VAL;
				Save_Coord(encoder_value);
				//Goes to Select Mode
				mode = SELECT;
			} else if (coord_name == SET) {
				//Write_LCD_Buffer((char*)"    Brush Moving    ", LCD_ROW_SIZE, ROW_4);
				//LCD_SendCommand(LCD_ADDR, ROW_4);
				if (Get_Direction_and_Diff() == 1)
				{
					//LCD_SendString(LCD_ADDR, "                    "); //TODO
					queue_var = SPACE_4_ROW_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					mode = CHECK_PEDAL;
					//send queue 35

				} else {
					//LCD_SendString(LCD_ADDR, "    Brush Moving    ");	//TODO
					queue_var = BRUSH_MOVING_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 35
					//Get_Direction_and_Diff();
					Lock_Handle();
					//Unlocks brush to move it
					Brush_Unlock();
					//Starts DAC
					//encoder_value = 0;
					HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
					//Goes to Brush Move mode
					mode = BRUSH_MOVE;
					//send queue 21
				}
			}
		}
	}
}

uint8_t Get_Coord_Size(char* coord_arr, double coord)
{
	char temp_buf[10];
	sprintf(temp_buf, "%6.1f", coord);

	for (int i = 0; i < sizeof(temp_buf); ++i) {
	  if (temp_buf[i] == 0x20) {
		  temp_buf[i] = '0';
	  }
	}
	coord_arr[0] = temp_buf[0];
	coord_arr[1] = temp_buf[1];
	coord_arr[2] = temp_buf[2];
	coord_arr[3] = temp_buf[3];
	coord_arr[4] = temp_buf[5];

	uint8_t temp = 0;
	for (uint8_t i = 0; i < COORD_SIZE; ++i) {
		if (coord_array[i] == '0') {
			temp++;
		} else {
			return COORD_SIZE - temp;
		}
	}
	return COORD_SIZE - temp;
}

/**
  * @brief	Checks pressed key.
  * @param	None
  * @retval None
  */
//uint8_t debug = 0;

void Check_Pressed_Key()
{
	char data = 0;

	if (!Empty(keypad_buf_length)) {
		data = Read_Keypad_Buffer(keypad_buffer);

		if (data == '*') {
			Reset_LCD_Pointers();
			//Write_LCD_Buffer((char*)"00000", COORD_SIZE, S_COORD_POS);
			//Write_LCD_Buffer((char*)"     Edit Mode      ", LCD_ROW_SIZE, ROW_4);
			queue_var = ZERO_S_COORD_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			queue_var = EDIT_MODE_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 22
			memset(coord_array, '0', COORD_SIZE);
			temp_coord = set_coord;
			set_coord = 0;
			coord_size = 0;
			//coord_size = Get_Coord_Size(coord_array, set_coord);
			number_accept_count = 0;
			//Goes to edit mode
			mode = EDIT;

		} else if (data == '#') {
			//Reset_LCD_Pointers();
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
			queue_var = SPACE_4_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 23
			//Goes pedal checking mode
			mode = CHECK_PEDAL;

		} else if (data == 'C') {
			//Reset_LCD_Pointers();
			//Write_LCD_Buffer((char*)"00000", COORD_SIZE, R_COORD_POS);
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_2);
			//Write_LCD_Buffer((char*)"    Callibration    ", LCD_ROW_SIZE, ROW_3);
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
			queue_var = ZERO_R_COORD_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			queue_var = SPACE_2_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			queue_var = CALLIBRATION_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			queue_var = SPACE_4_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 24
			memset(coord_array, '0', COORD_SIZE);
			//coord_size = Get_Coord_Size(coord_array, real_coord);
			temp_coord = real_coord;
			real_coord = 0;
			coord_size = 0;
			number_accept_count = 0;
			//Goes callibration mode
			mode = CALLIBRATION;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//					Brush Moving Mode                                        //
///////////////////////////////////////////////////////////////////////////////
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
	if (dir == FORWARD) {
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port,	Brush_Back_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin, GPIO_PIN_RESET);
		// /HAL_GPIO_WritePin(Brush_Back_GPIO_Port,	Brush_Back_Pin, GPIO_PIN_SET);
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
void Change_Speed(uint16_t *sp, uint8_t ramp)
{
	if (ramp == RAMP_UP) {
		if (*sp <= MAX_DAC_VALUE) {
			*sp = *sp + RAMP_UP_VAL;
			if (*sp > MAX_DAC_VALUE) {
				*sp = MAX_DAC_VALUE;
			}
		}
	} else if (ramp == RAMP_DOWN) {
		if (*sp >= (MIN_SPEED + RAMP_DOWN_VAL)) {
			*sp = *sp - RAMP_DOWN_VAL;
			//if (*speed < 0) {
			//	*speed = 0;
			//}
		}
	}
	/* Changes the speed by changing PWM duty cycle or DAC value */
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, *sp);
}

/**
  * @brief	Unlocks the brush to move it.
  * @param	None
  * @retval None
  */
void Brush_Unlock()
{
	 HAL_GPIO_WritePin(Brush_Lock_GPIO_Port, Brush_Lock_Pin, GPIO_PIN_RESET); // /tormuz
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
void Save_Coord(uint32_t coord)
{
	/* To write to the Backup register user needs to
	 * Unlock the Backup register to access it
	 * Write value to the Backup Register
	 * Lock the backup register
	 */
	//double f_coord = coord * ONE_ROTATION_TICK;
	//uint32_t r_coord = f_coord;

	/*set the DBP bit the Power Control
	Register (PWR_CR) to enable access to the Backup
	registers and RTC.*/
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, coord);
	HAL_PWR_DisableBkUpAccess();
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		 if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
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
uint32_t Read_Coord()
{
	uint32_t data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if (data == 0xffffffff)
	{
		return 0;
	}
	return data;
}

void Print_Current_Coord()
{
	if (print_real_coord_time == TIMEOUT_PRINT_REAL) {
		print_real_coord_time = 0;
		real_coord = (double)encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;
		//Reset_LCD_Pointers();
		Print_Coord(real_coord, REAL);
	}

	if (lcd_timeout == LCD_TIMEOUT) {
		lcd_timeout = 0;
		LCD_Write(LCD_ADDR);
	}
}

/**
  * @brief	Moves the brush in the determined direction.
  * 		and at the specified speed.
  * @param	None
  * @retval None
  */
int32_t encoder1 = 0;
int32_t encoder2 = 0;
int32_t encoder3 = 0;

uint8_t counter = 0;

void Ramp_Down(uint16_t* current_speed, uint16_t max_speed, uint16_t min_speed, uint8_t avg, uint8_t diff)
{
	if (abs(encoder_value - previous_encoder_value) >= (diff-1))
	{
		counter++;
		*current_speed = *current_speed - ((max_speed - min_speed) / avg);

		if (*current_speed < min_speed)
		{
			*current_speed = min_speed;
		}
		previous_encoder_value = encoder_value;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, *current_speed);
	}
}

uint8_t Get_Status(uint8_t dir)
{
	if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 0)
	{
		Save_Coord(encoder_value);
	}
	Print_Current_Coord();	//print current real coord
	if ((dir == FORWARD) && (encoder_value >= HARD_LIMIT_UP_IN_TICK))
	{
		arrange_out = 1;
		return 1;
	} else if ((dir == BACK) && (encoder_value <= LIMIT_DOWN_IN_TICK)) {
		arrange_out = 1;
		return 1;
	}
	return 0;
}

void Move_Brush()
{
	print_real_coord_time = 0;
	is_move = 0;
	arrange_out = 0;

	if (direction == FORWARD) //tesoghakan dashtic hervanum e ays depqum
	{
		if (is_min_speed == 0)
		{
			speed = MAX_SPEED;
			Set_Inverter(FORWARD, speed);

			while(encoder_value < set_tick)
			{
				if (Get_Status(FORWARD) == 1) break;
			}

			if (arrange_out == 0)
			{
				previous_encoder_value = encoder_value;

				while(encoder_value < (set_tick + EXTRA_COORD))
				{
					Ramp_Down(&speed, MAX_SPEED, MID_SPEED, AVG_COUNT, 100);
					if (Get_Status(FORWARD) == 1) break;
				}
				speed = MID_SPEED;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
				Set_Inverter(STOP, 0);
			}

			if (arrange_out == 0)
			{
				Set_Inverter(BACK, speed);

				previous_encoder_value = encoder_value;

				while(encoder_value > (set_tick + DELTA))
				{
					Ramp_Down(&speed, MID_SPEED, MIN_SPEED, AVG_COUNT, 100);
					if (Get_Status(BACK) == 1) break;
				}
				speed = MIN_SPEED;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
			}

		} else {

			speed = MIN_SPEED;
			Set_Inverter(FORWARD, speed);

			while(encoder_value < (set_tick + EXTRA_COORD))
			{
				if (Get_Status(FORWARD) == 1) break;
			}

			Set_Inverter(STOP, 0);

			if (arrange_out == 0)
			{
				Set_Inverter(BACK, speed);

				while(encoder_value > (set_tick + DELTA))
				{
					if (Get_Status(BACK) == 1) break;
				}
			}
		}
	} else if (direction == BACK)
	{
		if (is_min_speed == 0)
		{
			speed = MAX_SPEED;
			Set_Inverter(BACK, speed);

			while(encoder_value > (set_tick + EXTRA_COORD))
			{
				if (Get_Status(BACK) == 1) break;
			}

			if (arrange_out == 0)
			{
				previous_encoder_value = encoder_value;

				while(encoder_value > (set_tick + EXTRA_COORD - 200))
				{
					Ramp_Down(&speed, MAX_SPEED, MID_SPEED, AVG_COUNT, 40);
					if (Get_Status(BACK) == 1) break;
				}
				speed = MID_SPEED;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
			}

			if (arrange_out == 0)
			{
				previous_encoder_value = encoder_value;

				while(encoder_value > (set_tick + DELTA))
				{
					Ramp_Down(&speed, MID_SPEED, MIN_SPEED, AVG_COUNT, 60);
					if (Get_Status(BACK) == 1) break;
				}
				speed = MIN_SPEED;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
			}

		} else {
			speed = MIN_SPEED;
			while(encoder_value > (set_tick + DELTA))
			{
				if (Get_Status(BACK) == 1) break;
			}
		}
	}

	encoder1 = encoder_value;
	Brush_Lock();
	encoder2 = encoder_value;
	Set_Inverter(STOP, 0);
	//encoder1 = encoder_value;
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

	//if there is no power, save coordinate
	if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 0)
	{
		Save_Coord(encoder_value);
	}
	print_real_coord_time = 0;
	is_move = 0;
	timeout_for_ramp = 0;
	time_for_change_ramp = 0;
	arrange_out = 0;
	//Reset_LCD_Pointers();

	//encoder2 = encoder_value;

	//encoder3 = encoder_value;
	//Turns off inverter
	HAL_Delay(1000);
	//Locks brush to fix it
	encoder3 = encoder_value;
	real_coord = (double)encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;

	//Saves real coordinate to backup register
	Save_Coord(encoder_value);
	//Prints real coordinate to LCD
	Print_Coord(real_coord, REAL);
	//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
	queue_var = SPACE_4_ROW_CMD;
	xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
	//send queue 25
	Unlock_Handle();
	//encoder_value = 0;
	//Goes to CHECK_PEDAL mode
	mode = CHECK_PEDAL;
}

///////////////////////////////////////////////////////////////////////////////
//					     CUTTING MODE										 //
///////////////////////////////////////////////////////////////////////////////
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
			 &input_state.pedal_cnt_for_st1, &input_state.pedal_is_pressed, 0);// 1 er darav 0

	Read_Pin(Hand_Catch_GPIO_Port, Hand_Catch_Pin, &input_state.hand_catch_cnt_for_st0,
 &input_state.hand_catch_cnt_for_st1,	&input_state.hand_catch_is_pressed, 0);
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
	if (pin_data == on_state) {
		//reset counter for state1
		*st1_counter = 0;

		// Increment State0 counter
		if (*st0_counter < DEBOUNCE_TIME+1) {
			(*st0_counter)++;
		}
		//if state0 counter is equal to DEBOUNCE_TIME, we consider that button is pressed.
		if (*st0_counter == DEBOUNCE_TIME) {
			*is_pressed = 1;
		}
	//if button not pressed
	} else {

		*st0_counter = 0;

		if (*st1_counter < DEBOUNCE_TIME+1) {
			(*st1_counter)++;
		}
		if (*st1_counter == DEBOUNCE_TIME) {
			if (GPIOx == Cutting_Buttons_GPIO_Port && GPIO_Pin == Cutting_Buttons_Pin)  {
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
	if (HAL_GPIO_ReadPin(Knife_Sensor1_GPIO_Port, Knife_Sensor1_Pin) == 0) { // / 1 er darav 0
		/* Read the knife sensors pins Knife_Sensor1, Knife_Sensor2  */
		return 1;
	}
	return 0;
}

void Air_Out_On()
{
	HAL_GPIO_WritePin(Air_Out_GPIO_Port, Air_Out_Pin, GPIO_PIN_RESET); // / set er darav reset
}

void Air_Out_Off()
{
	HAL_GPIO_WritePin(Air_Out_GPIO_Port, Air_Out_Pin, GPIO_PIN_SET); // / reset er darav set
}

void Check_Pedal()
{
	//if pedal is pressed
	if (input_state.pedal_is_pressed == 1) {

		pedal_is_pressed = 1;
		Lock_Handle();
		Air_Out_Off();

		if (delay_for_cutting_buttons == TIMEOUT_TO_ACTIVATE_CUTTING_BUTTON) {

			//Activates cuttings button
			Cutting_Button_On();

			if ((input_state.cut_is_pressed == 1) && (cut_is_done == 0)) {

				if (delay_for_cutting == TIMEOUT_TO_CUT) {

					//Reads knife sensors
					if (Read_Knife_Sensors() == 1) {

						//Cuts the paper
						Cutting_On();
						temp = 1;
						if (old_temp != temp) {
							//Reset_LCD_Pointers();
							//Write_LCD_Buffer((char*)"      Cutting       ", LCD_ROW_SIZE, ROW_4);
							queue_var = CUTTING_CMD;
							xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
							//send queue 26
						}

					} else {
						Cutting_Off();
						cut_is_done = 1;
						temp = 2;
						if (old_temp != temp) {
							//Reset_LCD_Pointers();
							//Write_LCD_Buffer((char*)"    Cut is done     ", LCD_ROW_SIZE, ROW_4);
							queue_var = CUT_IS_DONE_CMD;
							xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
							//send queue 27
						}
					}
				}
			} else if ((input_state.cut_is_pressed == 0) && (cut_is_done == 0)) {
				Cutting_Off();
				delay_for_cutting = 0;
				temp = 3;
				if (old_temp != temp) {
					//Reset_LCD_Pointers();
					//Write_LCD_Buffer((char*)"  Allowed cutting   ", LCD_ROW_SIZE, ROW_4);
					queue_var = ALLOWED_CUTTING_CMD;
					xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
					//send queue 28
				}
			}
		}
	} else {
		if (pedal_is_pressed == 1) {
			old_temp = temp = 0;
			pedal_is_pressed = 0;
			delay_for_cutting_buttons = 0;
			delay_for_cutting = 0;
			Cutting_Button_Off();
			Air_Out_On();
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
			queue_var = SPACE_4_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 29
			Unlock_Handle();
		}
		if (keypad_timeout == KEYPAD_TIMEOUT) {
			keypad_timeout = 0;
			Read_Keypad();
		}
		if (!Empty(keypad_buf_length)) {
			uint8_t data = Read_Keypad_Buffer(keypad_buffer);

			//If pressed 'C' key goes to the CALLIBRATION mode
			if (data == '*') {
				mode = SELECT;
				//Reset_LCD_Pointers();
				//Write_LCD_Buffer((char*)" *-Edit #-Cut C-Cal ", LCD_ROW_SIZE, ROW_4);
				queue_var = MAIN_MENU_CMD;
				xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
				//send queue 30
			}
		}
		if (input_state.hand_catch_is_pressed == 1)
		{
			print_real_coord_time = 0;
			//initial_coord = real_coord;
			//encoder_value = 0;
			Brush_Unlock();
			//Write_LCD_Buffer((char*)"   Hand catching    ", LCD_ROW_SIZE, ROW_4);
			queue_var = HAND_CATCHING_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 31
			mode = HAND_CATCH;

		}
	}
	old_temp = temp;
}

///////////////////////////////////////////////////////////////////////////////
//							HAND CATCH										 //
///////////////////////////////////////////////////////////////////////////////
char temp_buf_enc[7];
char current_coord[6];

void Print_Coord(double r_coord, uint8_t coord_name)
{
	memset(temp_buf_enc, 0x00, 7);

	sprintf(current_coord, "%6.1f", r_coord);
	sprintf(temp_buf_enc, "%ld", encoder_value);

	for (int i = 0; i < sizeof(current_coord); ++i) {
	  if (current_coord[i] == 0x20) {
		  current_coord[i] = '0';
	  }
	}

	for (int i = 0; i < sizeof(temp_buf_enc); ++i) {
	  if (temp_buf_enc[i] == 0) {
		  temp_buf_enc[i] = 0x20;
	  }
	}
	temp_buf_enc[7] = '\0';
	current_coord[6] = '\0';

	Reset_LCD_Pointers();
	if (coord_name == REAL) {
		queue_var = REAL_CMD;
		xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
		queue_var = CURRENT_REAL_COORD_CMD;
		xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
		queue_var = ENCODER_VAL_CMD;
		xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
		//Write_LCD_Buffer((char*)"Real  ", sizeof("Real  "), ROW_1);
		//Write_LCD_Buffer(temp_buf, COORD_SIZE_WITH_POINT, R_COORD_POS);
		//Write_LCD_Buffer(temp_buf_enc, 7, ROW_3);
		//send queue 31
	} else {
		queue_var = SET_CMD;
		xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
		queue_var = CURRENT_SET_COORD_CMD;
		xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
		//Write_LCD_Buffer((char*)"Set   ", sizeof("Set   "), ROW_2);
		//Write_LCD_Buffer(temp_buf, COORD_SIZE_WITH_POINT, S_COORD_POS);
		//send queue 32
	}
}

void Check_Hand_Catch()
{
	if (input_state.hand_catch_is_pressed == 1) {
		hand_catch_detected = 1;

		//if there is no power, save coordinate
		if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 0)
		{
			Save_Coord(encoder_value);
		}

		if (print_real_coord_time == TIMEOUT_PRINT_REAL) {
			print_real_coord_time = 0;
			real_coord = (double)encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;
			//Reset_LCD_Pointers();
			Print_Coord(real_coord, REAL);
		}
	} else {
		if (hand_catch_detected == 1) {
			hand_catch_detected = 0;
			Brush_Lock();
			real_coord = (double)encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;

			//Prints real coordinate to LCD
			Print_Coord(real_coord, REAL);
			//Saves real coord to backup register
			Save_Coord(encoder_value);
			//Resets encoder value
			//encoder_value = 0;
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);
			queue_var = SPACE_4_ROW_CMD;
			xQueueSend(myQueue01Handle,( void * ) &queue_var, 10);
			//send queue 33
			mode = CHECK_PEDAL;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//								Main Task									 //
///////////////////////////////////////////////////////////////////////////////

uint8_t data = 0;

void Main_Task()
{
	#ifndef TEST
		//if (lcd_timeout == LCD_TIMEOUT) {
		//	lcd_timeout = 0;
		//	LCD_Write(LCD_ADDR);
		//}
		state_machine();
	#else
		if (keypad_timeout == KEYPAD_TIMEOUT) {
			keypad_timeout = 0;
			Read_Keypad();
		}

		if (!Empty(keypad_buf_length)) {
			data = Read_Keypad_Buffer(keypad_buffer);
		}

		speed = 1600;
		if (data == 'A')
		{
			data = 0;

			//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			Brush_Unlock();
			encoder_value = 0;
			//Set_Inverter(FORWARD, speed);
			while(encoder_value < 3000)
			{

			}
			//Brush_Lock();
			//Set_Inverter(STOP, speed);
			//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);


		} else if (data == 'B')
		{
			data = 0;
			//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			Brush_Unlock();
			encoder_value = 3000;
			//Set_Inverter(BACK, speed);
			while(encoder_value > 0)
			{

			}
			//Brush_Lock();
			//Set_Inverter(STOP, speed);
			//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

		}
	#endif
}

void state_machine()
{
	switch(mode)
	{

		case SELECT:
		{
			if (keypad_timeout == KEYPAD_TIMEOUT) {
				keypad_timeout = 0;
				Read_Keypad();
			}
			pressed_count = 0;
			Check_Pressed_Key();
			break;
		}
		case CALLIBRATION:
		{
			if (keypad_timeout == KEYPAD_TIMEOUT) {
				keypad_timeout = 0;
				Read_Keypad();
			}
			Collects_Digits(REAL);
			break;
		}
		case EDIT:
		{
			if (keypad_timeout == KEYPAD_TIMEOUT) {
				keypad_timeout = 0;
				Read_Keypad();
			}
			Collects_Digits(SET);
			break;
		}
		case BRUSH_MOVE:
		{
			//if board is powered and brush is unlocked
			if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 1 &&
					HAL_GPIO_ReadPin(Brush_Lock_GPIO_Port, Brush_Lock_Pin) == 0)
			{
				Move_Brush();
			}
			break;
		}
		case CHECK_PEDAL:
		{
			if (input_timeout == INPUT_TIMEOUT) {
				input_timeout = 0;
				Read_Inputs();
			}
			Check_Pedal();
			break;
		}
		case HAND_CATCH:
		{
			if (input_timeout == INPUT_TIMEOUT) {
				input_timeout = 0;
				Read_Inputs();
			}
			Check_Hand_Catch();
			break;
		}
	}
}
