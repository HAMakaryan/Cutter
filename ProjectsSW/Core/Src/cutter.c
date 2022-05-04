/*
 * cutter.c
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "stdio.h"
#include <stdlib.h>
#include "main.h"
#include "cutter.h"
#include "math.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static uint8_t coord_size = 0;
static uint8_t mode = MANUAL;

static uint8_t lcd_buf_length = 0;
static uint8_t lcd_write_pnt = 0;
static uint8_t lcd_read_pnt = 0;
static uint8_t pos[ROW_SIZE] = { 1, 2, 4, 8 };
static uint8_t keypad_buf_length = 0;
static uint8_t keypad_wr_pnt = 0;
static uint8_t keypad_rd_pnt = 0;
static char keypad_buffer[KEYPAD_BUF_SIZE];

static uint8_t state = IDLE;
static uint8_t new_pressed_key = 0;
static uint8_t old_pressed_key = 0;
static uint8_t debounce = 0;
static uint8_t row_key = 0;
static uint8_t col_key = 0;

static uint8_t direction = 0;
static uint8_t arrange_out = 0;

GPIO_TypeDef *row_gpio_port[ROW_SIZE] = { Row0_GPIO_Port, Row1_GPIO_Port,
Row2_GPIO_Port, Row3_GPIO_Port };
GPIO_TypeDef *col_gpio_port[COL_SIZE] = { Col0_GPIO_Port, Col1_GPIO_Port,
Col2_GPIO_Port, Col3_GPIO_Port };
uint16_t row_gpio_pin[ROW_SIZE] = { Row0_Pin, Row1_Pin, Row2_Pin, Row3_Pin };
uint16_t col_gpio_pin[COL_SIZE] = { Col0_Pin, Col1_Pin, Col2_Pin, Col3_Pin };

uint8_t lcd_timeout = 0;
uint8_t keypad_timeout = 0;
uint8_t input_timeout = 0;

int32_t encoder_value = 0;
int32_t previous_encoder_value = 0;
Input_State input_state;
uint16_t delay_for_cutting_buttons = 0;
uint16_t delay_for_cutting = 0;
uint16_t print_real_coord_time = 0;
uint16_t back_forward_button_timeout = 0;

char coord_array[COORD_SIZE];
double set_coord = 0;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;
extern DAC_HandleTypeDef hdac;
extern osMessageQueueId_t myQueue01Handle;
extern double real_coord;
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
void LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags) {
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr),
			HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
}

/**
 * @brief	Sends a command to LCD with blocking mode.
 * @param	LCD address
 * @param	Command to be sent
 * @retval	None
 */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
	LCD_SendInternal(lcd_addr, cmd, 0);
}

/**
 * @brief	Sends a data to LCD with blocking mode.
 * @param	LCD address
 * @param	Data to be sent
 * @retval	None
 */
void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
	LCD_SendInternal(lcd_addr, data, PIN_RS);
}

/**
 * @brief	Sends a string to LCD with blocking mode.
 * @param	LCD address
 * @param	String to be sent
 * @retval	None
 */
void LCD_SendString(uint8_t lcd_addr, char *str) {
	while (*str) {
		LCD_SendData(lcd_addr, (uint8_t) (*str));
		str++;
	}
}

/**
 * @brief	Initializes the LCD(16x4) module using I2C peripheral.
 * @param	lcd address
 * @retval	None
 */
void LCD_Init(uint8_t lcd_addr) {
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
uint8_t Full(uint8_t buff_size) {
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
uint8_t Empty(uint8_t buff_size) {
	if (buff_size == 0) {
		return 1;
	} else {
		return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
//									KEYPAD									 //
///////////////////////////////////////////////////////////////////////////////
/**
 * @brief	Initializes the keypad(4x4).
 * @param	None
 * @retval None
 */
void Keypad_Init() {
	//set all rows and columns
	HAL_GPIO_WritePin(GPIOF, Row0_Pin | Row1_Pin | Col0_Pin | Col1_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, Row2_Pin | Col3_Pin, GPIO_PIN_SET);
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
void Set_Columns_Output() {
	GPIOF->MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER15_0;
	GPIOE->MODER |= GPIO_MODER_MODER0_0;
	GPIOG->MODER |= GPIO_MODER_MODER8_0;
}

/**
 * @brief	Sets all columns as input.
 * @param	None
 * @retval None
 */
void Set_Columns_Input() {
	GPIOF->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER15_0);
	GPIOE->MODER &= ~(GPIO_MODER_MODER0_0);
	GPIOG->MODER &= ~(GPIO_MODER_MODER8_0);
}

/**
 * @brief	Sets all rows as output.
 * @param	None
 * @retval None
 */
void Set_Rows_Output() {
	GPIOF->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
	GPIOG->MODER |= GPIO_MODER_MODER14_0;
	GPIOD->MODER |= GPIO_MODER_MODER10_0;
}

/**
 * @brief	Sets all rows as input.
 * @param	None
 * @retval None
 */
void Set_Rows_Input() {
	GPIOF->MODER &= ~(GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0);
	GPIOG->MODER &= ~(GPIO_MODER_MODER14_0);
	GPIOD->MODER &= ~(GPIO_MODER_MODER10_0);
}

/**
 * @brief	Reads all rows.
 * @param	Pressed rows counter
 * @retval None
 */
void Read_Rows(uint8_t *row_counter) {
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
void Read_Columns(uint8_t *col_counter) {
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
void Keypad_Write_Buffer(char data) {
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
char Read_Keypad_Buffer(char *buffer) {
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
char Convert_Key_to_Char(uint8_t key) {
	switch (key) {
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
void Read_Keypad() {
	uint8_t row_pressed_counter = 0;
	uint8_t col_pressed_counter = 0;

	switch (state) {
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
		} else if (col_pressed_counter == SINGLE_KEY
				&& row_pressed_counter == SINGLE_KEY) {
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
		if (row_pressed_counter == 0 && col_pressed_counter == 0) {
			debounce++;

			//After debounce time goes "IDLE" state
			if (debounce == DEBOUNCE_TIME) {
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
void Lock_Handle() {
	HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_SET);
}

void Unlock_Handle() {
	HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Writes data to LCD buffer.
 * @param	Data to be written
 * @param  Data size
 * @param  Cursor position
 * @retval None
 */
void Write_LCD_Buffer(char *buf, uint8_t size, uint8_t cursor) {
	uint8_t lcd_buf[30];

	if (cursor != ROW_3) {
		for (uint8_t i = 0; i < size; ++i) {
			lcd_buf[i] = buf[i];
		}

		if (buf[4] != '.') {
			if (cursor == 0xDA || cursor == R_COORD_POS) {
				uint16_t temp = 0;
				temp = lcd_buf[4];
				lcd_buf[4] = '.';
				lcd_buf[5] = temp;
				lcd_buf[6] = '\0';
			}
		}
		LCD_SendString(LCD_ADDR, (char*) lcd_buf);

	} else {

		LCD_SendString(LCD_ADDR, buf);
	}
}

/**
 * @brief	Creates set coordinate value from collected digits.
 * @param	Buffer with digits
 * @retval Set coordinate
 */
double Create_Number(char *buf) {
	double coord = ((buf[0] - '0') * 1000) + ((buf[1] - '0') * 100)
			+ ((buf[2] - '0') * 10) + (buf[3] - '0') + ((buf[4] - '0') * 0.1);

	return coord;
}

void Reset_LCD_Pointers() {
	lcd_write_pnt = lcd_read_pnt = 0;
	lcd_buf_length = 0;
}

uint32_t set_tick = 0;
uint32_t tick_difference = 0;

uint8_t Check_Coord_and_Get_Ticks(float coord) {
	set_tick = round((double) coord * ONE_ROTATION_TICK / ONE_ROTATION_VAL);

	if (set_tick == encoder_value) {
		return COORD_IS_NOT_CHANGED;
	}

	int32_t tick_diff = encoder_value - set_tick;
	tick_difference = abs(tick_diff);

	if (abs(tick_diff) < 5) {
		return REAL_SET_COORDS_DIFF_LOW;
	}

	//Defines direction
	if (tick_diff < 0) {
		direction = BACK;
	} else {
		direction = FORWARD;
	}
	return 0;
}

uint8_t debug = 0;
uint8_t pressed_count = 0;
uint8_t queue_var = 0;
uint8_t pressed_hash_count = 0;

uint8_t Modify_Coord(char *coord_array, double *coord, double coord_pr_val,
		char key) {
	static double temp_coord = 0;

	if (key == 'B') {
		*coord = coord_pr_val;
		temp_coord = 0;
		coord_size = 0;
		pressed_hash_count = 0;
		return PRESSED_BACK_KEY;

	} else if (key >= '0' && key <= '9') {//add digit to coord array(by shifting right to left)

		if (pressed_hash_count == 0) {

			uint8_t all_zero = 0;

			if (key == '0') {
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
						coord_array[i] = coord_array[i + 1];
					}
					coord_array[COORD_SIZE - 1] = key;
				}
			}
		}
	} else if (key == '*') {	//delete last digit from coord array

		if (pressed_hash_count == 0) {

			if (coord_size > 0) {
				coord_size--;

				for (uint8_t i = 0; i < COORD_SIZE - 1; ++i) {
					coord_array[COORD_SIZE - i - 1] = coord_array[COORD_SIZE - i
							- 2];
				}
				coord_array[0] = '0';
			}
		} else if (pressed_hash_count == 1)
		{
			pressed_hash_count = 0;
			if (mode == EDIT_SET_COORD)
			{
				queue_var = SAVE_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			}
			queue_var = CURSOR_BLINKING_ON;
			xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
		}
	} else if (key == '#') {

		pressed_hash_count++;

		if (pressed_hash_count == 1) {

			//Creates number from collected digits
			temp_coord = Create_Number(coord_array);

			if (temp_coord < LIMIT_DOWN || temp_coord > SOFT_LIMIT_UP) {

				char temp_buf[10];

				if (temp_coord < LIMIT_DOWN) {
					sprintf(temp_buf, "%6.1f", (double) LIMIT_DOWN);
				} else if (temp_coord > SOFT_LIMIT_UP) {
					sprintf(temp_buf, "%6.1f", (double) SOFT_LIMIT_UP);
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

				if (temp_coord < LIMIT_DOWN) {
					temp_coord = LIMIT_DOWN;

				} else if (temp_coord > SOFT_LIMIT_UP) {
					temp_coord = SOFT_LIMIT_UP;
				}
				char temp_array[7];
				coord_size = Get_Coord_Size(temp_array, temp_coord);
			}
			if (mode == EDIT_SET_COORD)
			{
				queue_var = GO_TO_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			}
			queue_var = CURSOR_BLINKING_OFF;
			xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
		}
		if (pressed_hash_count == 2) {
			pressed_hash_count = 0;

			*coord = temp_coord;
			temp_coord = 0;
			coord_size = 0;

			return PRESSED_HASH_KEY_TWO_TIME;
		}
	}
	return 0;
}

uint8_t Get_Coord_Size(char *coord_arr, double coord) {
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
 * @retval Pressed key
 */
char Read_Pressed_Key() {
	char pressed_key = 0;

	if (!Empty(keypad_buf_length)) {
		pressed_key = Read_Keypad_Buffer(keypad_buffer);
	}
	return pressed_key;
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
void Set_Inverter(uint8_t dir, uint16_t speed) {
	if (dir == FORWARD) {
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port, Brush_Back_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin,
				GPIO_PIN_RESET);
		// /HAL_GPIO_WritePin(Brush_Back_GPIO_Port,	Brush_Back_Pin, GPIO_PIN_SET);
	} else if (dir == BACK) {
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port, Brush_Back_Pin, GPIO_PIN_RESET);
	} else if (dir == STOP) {
		HAL_GPIO_WritePin(Brush_Forward_GPIO_Port, Brush_Forward_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(Brush_Back_GPIO_Port, Brush_Back_Pin, GPIO_PIN_SET);
	}
	//Sets DAC value
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
}

/**
 * @brief	Unlocks the brush to move it.
 * @param	None
 * @retval None
 */
void Brush_Unlock() {
	HAL_GPIO_WritePin(Brush_Lock_GPIO_Port, Brush_Lock_Pin, GPIO_PIN_RESET); // /tormuz
}

/**
 * @brief	Locks the brush to fix it.
 * @param	None
 * @retval None
 */
void Brush_Lock() {
	HAL_GPIO_WritePin(Brush_Lock_GPIO_Port, Brush_Lock_Pin, GPIO_PIN_SET);
}

/**
 * @brief	Saves the brush coordinate to the Backup register.
 * @param	The address of the Backup register
 * @param	The value to be saved
 * @retval None
 */
void Save_Coord(uint32_t coord) {
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
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
uint32_t Read_Coord() {
	uint32_t data = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if (data == 0xffffffff) {
		return 0;
	}
	return data;
}

void Print_Current_Coord() {
	if (print_real_coord_time == TIMEOUT_PRINT_REAL) {
		print_real_coord_time = 0;

		real_coord = (double) encoder_value * ONE_ROTATION_VAL
				/ ONE_ROTATION_TICK;
		Print_Coord(real_coord, REAL);
	}
}

uint8_t Get_Status(uint8_t dir) {
	if ((dir == BACK) && (encoder_value >= HARD_LIMIT_UP_IN_TICK )) {
		arrange_out = 1;
		return 1;
	} else if ((dir == FORWARD) && (encoder_value <= LIMIT_DOWN_IN_TICK )) {
		arrange_out = 1;
		return 1;
	}
	return 0;
}

void set_debug_pins(uint8_t data) {
	if ((data & 0x01) == 0x01) {
		HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DB0_GPIO_Port, DB0_Pin, GPIO_PIN_RESET);
	}
	if ((data & 0x02) == 0x02) {
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DB1_GPIO_Port, DB1_Pin, GPIO_PIN_RESET);
	}
	if ((data & 0x04) == 0x04) {
		HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DB2_GPIO_Port, DB2_Pin, GPIO_PIN_RESET);
	}
	if ((data & 0x08) == 0x08) {
		HAL_GPIO_WritePin(DB3_GPIO_Port, DB3_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DB3_GPIO_Port, DB3_Pin, GPIO_PIN_RESET);
	}
	if ((data & 0x10) == 0x10) {
		HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(DB4_GPIO_Port, DB4_Pin, GPIO_PIN_RESET);
	}
}

uint16_t speed = MIN_SPEED;

uint8_t Process_Brush_Moving() {
	print_real_coord_time = 0;
	arrange_out = 0;

	//Unlocks brush to move it
	Brush_Unlock();
	//Starts DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

	if (direction == BACK) //tesoghakan dashtic sanr@ hervanum e ays depqum
	{
		uint8_t temp = 0;

		if (tick_difference < 417) {
			speed = MIN_SPEED;
			temp = 1;

		} else if (tick_difference < 2086) {
			speed = MID_SPEED;
			temp = 1;

		} else {
			speed = MAX_SPEED;
			temp = 0;
		}

		Set_Inverter(BACK, speed);

		while (encoder_value < (set_tick + EXTRA_COORD)) {
			if ((encoder_value >= (set_tick - 1500)) && (temp == 0)) //heranalu jamanak 1500
					{
				speed = MID_SPEED;
				Set_Inverter(BACK, speed);
				temp = 1;
			}

			Print_Current_Coord();	//print current real coord
			if (Get_Status(BACK) == 1) {
				break;
			}
		}
		Set_Inverter(STOP, 0);

		if (arrange_out == 0) {
			speed = MIN_SPEED;
			Set_Inverter(FORWARD, speed);

			while (encoder_value > (set_tick + DELTA)) {
				Print_Current_Coord();	//print current real coord
				if (Get_Status(FORWARD) == 1) {
					break;
				}
			}
		}
	} else if (direction == FORWARD) {	//ays depqum sanr@ motenum e mez
		uint8_t temp = 0;

		if (tick_difference < 417) {
			speed = MIN_SPEED;
			temp = 2;

		} else if (tick_difference < 2086) {
			speed = MID_SPEED;
			temp = 1;

		} else {
			speed = MAX_SPEED;
			temp = 0;
		}
		Set_Inverter(FORWARD, speed);

		while (encoder_value > (set_tick + DELTA)) {
			if ((encoder_value <= (set_tick + 100)) && (temp == 1)) {
				speed = MIN_SPEED;
				Set_Inverter(FORWARD, speed);
				temp = 2;

			} else if ((encoder_value <= (set_tick + 100 + 1500))
					&& (temp == 0)) //motenalu jamanak 1500 + 100
					{
				speed = MID_SPEED;
				Set_Inverter(FORWARD, speed);
				temp = 1;
			}

			Print_Current_Coord();	//print current real coord
			if (Get_Status(FORWARD) == 1) {
				break;
			}
		}
	}
	Brush_Lock();
	Set_Inverter(STOP, 0);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	set_debug_pins(0x00);

	print_real_coord_time = 0;
	arrange_out = 0;

	HAL_Delay(1000);

	real_coord = (double) encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;

	//Saves real coordinate to backup register
	Save_Coord(encoder_value);
	//Prints real coordinate to LCD
	Print_Coord(real_coord, REAL);

	return BRUSH_MOVING_ENDED;
}

uint32_t Process_Brush_Moving_With_Buttons(uint8_t direction) {
	print_real_coord_time = 0;
	back_forward_button_timeout = 0;
	arrange_out = 0;

	//Unlocks brush to move it
	Brush_Unlock();
	//Starts DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	speed = MID_SPEED;

	if (direction == FORWARD)
	{
		Set_Inverter(FORWARD, speed);

		while (input_state.forward_is_pressed == 1) {
			if (input_timeout == INPUT_TIMEOUT) {
				input_timeout = 0;
				Read_Inputs();
			}
			//Change motor speed after BACK_FORWARD_BUTTON_PRESSED_TIME
			if (back_forward_button_timeout == BACK_FORWARD_BUTTON_PRESSED_TIME)
			{
				back_forward_button_timeout++;
				speed = MAX_SPEED;
				Set_Inverter(FORWARD, speed);
			}
			Print_Current_Coord();	//print current real coord
			if (Get_Status(FORWARD) == 1) {
				break;
			}
		}
	} else if (direction == BACK) {
		Set_Inverter(BACK, speed);

		while (input_state.back_is_pressed == 1) {
			if (input_timeout == INPUT_TIMEOUT) {
				input_timeout = 0;
				Read_Inputs();
			}
			//Change motor speed after BACK_FORWARD_BUTTON_PRESSED_TIME
			if (back_forward_button_timeout == BACK_FORWARD_BUTTON_PRESSED_TIME)
			{
				back_forward_button_timeout++;
				speed = MAX_SPEED;
				Set_Inverter(BACK, speed);
			}
			Print_Current_Coord();	//print current real coord
			if (Get_Status(BACK) == 1) {
				break;
			}
		}
	}
	Brush_Lock();
	Set_Inverter(STOP, 0);
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
	set_debug_pins(0x00);

	print_real_coord_time = 0;
	arrange_out = 0;

	HAL_Delay(1000);

	real_coord = (double) encoder_value * ONE_ROTATION_VAL / ONE_ROTATION_TICK;

	//Saves real coordinate to backup register
	Save_Coord(encoder_value);
	//Prints real coordinate to LCD
	Print_Coord(real_coord, REAL);

	return BRUSH_MOVING_ENDED;
}

///////////////////////////////////////////////////////////////////////////////
//					     CUTTING MODE										 //
///////////////////////////////////////////////////////////////////////////////
/**
 * @brief	Reads specified inputs.
 * @param	None
 * @retval None
 */
void Read_Inputs(void) {
	Read_Pin(Cutting_Buttons_GPIO_Port, Cutting_Buttons_Pin,
			&input_state.cut_cnt_for_st0, &input_state.cut_cnt_for_st1,
			&input_state.cut_is_pressed, 0);

	Read_Pin(Pedal_In_GPIO_Port, Pedal_In_Pin, &input_state.pedal_cnt_for_st0,
			&input_state.pedal_cnt_for_st1, &input_state.pedal_is_pressed, 0);// 1 er darav 0

	Read_Pin(Hand_Catch_GPIO_Port, Hand_Catch_Pin,
			&input_state.hand_catch_cnt_for_st0,
			&input_state.hand_catch_cnt_for_st1,
			&input_state.hand_catch_is_pressed, 0);

	Read_Pin(Move_Brush_Forward_GPIO_Port, Move_Brush_Forward_Pin,
			&input_state.forward_cnt_for_st0, &input_state.forward_cnt_for_st1,
			&input_state.forward_is_pressed, 0);

	Read_Pin(Move_Brush_Back_GPIO_Port, Move_Brush_Back_Pin,
			&input_state.back_cnt_for_st0, &input_state.back_cnt_for_st1,
			&input_state.back_is_pressed, 0);
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
void Read_Pin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *st0_counter,
		uint8_t *st1_counter, uint8_t *is_pressed, uint8_t on_state) {
	uint8_t pin_data = 0xFF;

	//read desired button pin
	pin_data = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	//if button is pressed
	if (pin_data == on_state) {
		//reset counter for state1
		*st1_counter = 0;

		// Increment State0 counter
		if (*st0_counter < DEBOUNCE_TIME + 1) {
			(*st0_counter)++;
		}
		//if state0 counter is equal to DEBOUNCE_TIME, we consider that button is pressed.
		if (*st0_counter == DEBOUNCE_TIME) {
			*is_pressed = 1;
		}
		//if button not pressed
	} else {

		*st0_counter = 0;

		if (*st1_counter < DEBOUNCE_TIME + 1) {
			(*st1_counter)++;
		}
		if (*st1_counter == DEBOUNCE_TIME) {
			*is_pressed = 0;
		}
	}
}

/**
 * @brief	Activates cutting buttons.
 * @param	None
 * @retval None
 */
void Cutting_Button_On() {
	HAL_GPIO_WritePin(Cutting_Buttons_Allow_GPIO_Port,
			Cutting_Buttons_Allow_Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Deactivate cutting buttons.
 * @param	None
 * @retval None
 */
void Cutting_Button_Off() {
	HAL_GPIO_WritePin(Cutting_Buttons_Allow_GPIO_Port,
			Cutting_Buttons_Allow_Pin, GPIO_PIN_SET);
}

/**
 * @brief	Turns on Cutting output (knife) to cut the paper.
 * @param	None
 * @retval None
 */
void Cutting_On() {
	HAL_GPIO_WritePin(Cutting_GPIO_Port, Cutting_Pin, GPIO_PIN_RESET);
}

/**
 * @brief	Turns off Cutting output (knife).
 * @param	None
 * @retval None
 */
void Cutting_Off(void) {
	HAL_GPIO_WritePin(Cutting_GPIO_Port, Cutting_Pin, GPIO_PIN_SET);
}

/**
 * @brief	Reads the sensors which are attached to the knife
 * 		to determine the position of the knife.
 * @param	None
 * @retval Can 0,1
 */
uint8_t Read_Knife_Sensors(void) {
	if (HAL_GPIO_ReadPin(Knife_Sensor1_GPIO_Port, Knife_Sensor1_Pin) == 0) { // / 1 er darav 0
		/* Read the knife sensors pins Knife_Sensor1, Knife_Sensor2  */
		return 1;
	}
	return 0;
}

void Air_Out_On() {
	HAL_GPIO_WritePin(Air_Out_GPIO_Port, Air_Out_Pin, GPIO_PIN_RESET); // / set er darav reset
}

void Air_Out_Off() {
	HAL_GPIO_WritePin(Air_Out_GPIO_Port, Air_Out_Pin, GPIO_PIN_SET); // / reset er darav set
}

uint8_t Process_Cutting() {

	static uint8_t pedal_is_pressed = 0;
	static uint8_t cut_is_done = 0;
	static uint8_t temp = 0;
	static uint8_t old_temp = 0;

	//if pedal is pressed
	if (input_state.pedal_is_pressed == 1) {

		pedal_is_pressed = 1;
		Air_Out_Off();

		if (delay_for_cutting_buttons == TIMEOUT_TO_ACTIVATE_CUTTING_BUTTON) {

			//Activates cuttings button
			Cutting_Button_On();

			if ((input_state.cut_is_pressed == 1) && (cut_is_done == 0)) {

				if (delay_for_cutting == TIMEOUT_TO_CUT) {

					Cutting_On();		//himq e dnum danaki sharjvelun

					//Reads knife sensors
					if (Read_Knife_Sensors() == 1) {
						temp = 1;
						if (old_temp != temp) {
							queue_var = CUTTING_CMD;
							xQueueSend(myQueue01Handle, (void* ) &queue_var,
									10);
						}

					} else {
						if (temp == 1) {

							Cutting_Off();
							cut_is_done = 1;//sa nshanakum e 1 shrjan katarel e danak@ yev chaqi vra e
							temp = 2;
							if (old_temp != temp) {
								queue_var = CUT_IS_DONE_CMD;
								xQueueSend(myQueue01Handle, (void* ) &queue_var,
										10);
							}
						}
					}
				}
			} else if ((input_state.cut_is_pressed == 0)) {
				cut_is_done = 0;
				Cutting_Off();
				delay_for_cutting = 0;
				temp = 3;
				if (old_temp != temp) {
					queue_var = ALLOWED_CUTTING_CMD;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
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

			return PEDAL_RELEASED;
		}
	}
	old_temp = temp;

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//							HAND CATCH										 //
///////////////////////////////////////////////////////////////////////////////

void Print_Coord(double r_coord, uint8_t coord_name) {
	if (coord_name == REAL) {
		queue_var = CURRENT_REAL_COORD_CMD;
		xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);

	}
}

uint8_t Process_Hand_Catching() {
	static uint8_t hand_catch_detected = 0;

	if (input_state.hand_catch_is_pressed == 1) {
		hand_catch_detected = 1;

		if (print_real_coord_time == TIMEOUT_PRINT_REAL) {
			print_real_coord_time = 0;
			real_coord = (double) encoder_value * ONE_ROTATION_VAL
					/ ONE_ROTATION_TICK;
			Print_Coord(real_coord, REAL);
		}
	} else {
		if (hand_catch_detected == 1) {
			hand_catch_detected = 0;
			Lock_Handle();
			Brush_Lock();
			real_coord = (double) encoder_value * ONE_ROTATION_VAL
					/ ONE_ROTATION_TICK;

			//Prints real coordinate to LCD
			Print_Coord(real_coord, REAL);
			//Saves real coord to backup register
			Save_Coord(encoder_value);

			return HAND_CATCH_RELEASED;
		}
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//								Main Task									 //
///////////////////////////////////////////////////////////////////////////////
void Main_Task() {
	state_machine();
}

void state_machine()
{
	uint8_t ret = 0;
	static char key = 0;
	static double previous_coord = 0;
	static uint8_t old_mode = 0;

	static uint8_t mode_from = MANUAL;

	if (input_timeout == INPUT_TIMEOUT)
	{
		input_timeout = 0;
		Read_Inputs();
	}

	send_messages_to_LCD(mode, &old_mode);

	switch (mode)
	{
		case MANUAL:
		{
			if (keypad_timeout == KEYPAD_TIMEOUT)
			{
				keypad_timeout = 0;
				Read_Keypad();
			}
			key = Read_Pressed_Key();

			// if not pressed any button, check pressed key
			if ((input_state.pedal_is_pressed == 0)
					&& (input_state.hand_catch_is_pressed == 0)
					&& (input_state.forward_is_pressed == 0)
					&& (input_state.back_is_pressed == 0))
			{
				// if pressed 0-9 digits go "EDIT_SET_COORD" mode
				if (key >= '0' && key <= '9')
				{
					previous_coord = set_coord;
					set_coord = 0;
					memset(coord_array, '0', 6);
					mode = EDIT_SET_COORD;

				} else if (key == '*')
				{
					mode = AUTO;	//TODO:

				} else if (key == 'D')
				{
					mode = MENU;
				}
			} else
			{
				if (input_state.pedal_is_pressed == 1)
				{
					if ((input_state.hand_catch_is_pressed == 0) && (input_state.forward_is_pressed == 0)
											&& (input_state.back_is_pressed == 0))
						mode = CUTTING;

				} else if (input_state.hand_catch_is_pressed == 1)
				{
					if ((input_state.pedal_is_pressed == 0) && (input_state.forward_is_pressed == 0)
											&& (input_state.back_is_pressed == 0))
					{
						Unlock_Handle();
						Brush_Unlock();
						mode = HAND_CATCH;
					}

				} else if (input_state.forward_is_pressed == 1)
				{
					if ((input_state.pedal_is_pressed == 0) && (input_state.hand_catch_is_pressed == 0)
												&& (input_state.back_is_pressed == 0))
					{
						direction = FORWARD;
						mode = BRUSH_MOVE_WITH_BUTTONS;
					}

				} else if (input_state.back_is_pressed == 1)
				{
					if ((input_state.pedal_is_pressed == 0) && (input_state.hand_catch_is_pressed == 0)
												&& (input_state.forward_is_pressed == 0))
					{
						direction = BACK;
						mode = BRUSH_MOVE_WITH_BUTTONS;
					}
				}
			}
			mode_from = MANUAL;
			break;
		}

		case MENU:
			if (keypad_timeout == KEYPAD_TIMEOUT)
			{
				keypad_timeout = 0;
				Read_Keypad();
			}
			key = Read_Pressed_Key();
			if (key == 'C')
			{
				previous_coord = real_coord;
				real_coord = 0;
				memset(coord_array, '0', 6);
				mode = CALLIBRATION;
			}

			if (key == 'B')
			{
				mode = MANUAL;
			}
			break;

		case EDIT_SET_COORD:
		{
			ret = Modify_Coord(coord_array, &set_coord, previous_coord, key);

			if (key != 0)
			{
				queue_var = SET_COORD_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			}

			if (ret == PRESSED_BACK_KEY)
			{
				mode = MANUAL;

			} else if (ret == PRESSED_HASH_KEY_TWO_TIME)
			{
				uint32_t ret = Check_Coord_and_Get_Ticks(set_coord);

				if ((ret != COORD_IS_NOT_CHANGED)
						&& (ret != REAL_SET_COORDS_DIFF_LOW))
				{
					mode = BRUSH_MOVING;
				} else
				{
					mode = MANUAL;
				}
			}
			if (keypad_timeout == KEYPAD_TIMEOUT)
			{
				keypad_timeout = 0;
				Read_Keypad();
			}
			key = Read_Pressed_Key();
			break;
		}
		case BRUSH_MOVING:
		{
			//if board is powered
			if (HAL_GPIO_ReadPin(Power_In_GPIO_Port, Power_In_Pin) == 1)
			{
				ret = Process_Brush_Moving();
				if (ret == BRUSH_MOVING_ENDED)
				{
					mode = MANUAL;
				}
			}
			break;
		}
		case CUTTING:
		{
			ret = Process_Cutting();
			if (ret == PEDAL_RELEASED)
			{
				mode = MANUAL;
			}
			break;
		}
		case HAND_CATCH:
		{
			ret = Process_Hand_Catching();
			if (ret == HAND_CATCH_RELEASED)
			{
				mode = mode_from;
				if (mode_from == CALLIBRATION)
				{
					previous_coord = real_coord;
					coord_size = Get_Coord_Size(coord_array, real_coord);
				}
			}
			break;
		}
		case BRUSH_MOVE_WITH_BUTTONS:
		{
			ret = Process_Brush_Moving_With_Buttons(direction);
			if (ret == BRUSH_MOVING_ENDED)
			{
				mode = mode_from;
				if (mode_from == CALLIBRATION)
				{
					previous_coord = real_coord;
					coord_size = Get_Coord_Size(coord_array, real_coord);
				}
			}
			break;
		}
		case CALLIBRATION:

			// while is not pressed #, user can change real coord by hand catch or by buttons
			if (pressed_hash_count == 0)
			{
				if (input_state.hand_catch_is_pressed == 1)
				{
					if ((input_state.forward_is_pressed == 0) && (input_state.back_is_pressed == 0))
					{
						Unlock_Handle();
						Brush_Unlock();
						mode = HAND_CATCH;
					}

				} else if (input_state.forward_is_pressed == 1)
				{
					if ((input_state.hand_catch_is_pressed == 0) && (input_state.back_is_pressed == 0))
					{
						direction = FORWARD;
						mode = BRUSH_MOVE_WITH_BUTTONS;
					}

				} else if (input_state.back_is_pressed == 1)
				{
					if ((input_state.hand_catch_is_pressed == 0) && (input_state.forward_is_pressed == 0))
					{
						direction = BACK;
						mode = BRUSH_MOVE_WITH_BUTTONS;
					}
				}
			}

			// if not pressed hand catch or move forward, back buttons, user can edit real coord
			if ((input_state.hand_catch_is_pressed == 0) && (input_state.forward_is_pressed == 0)
					&& (input_state.back_is_pressed == 0))
			{
				if (key != 0)
				{
					queue_var = REAL_COORD_CMD;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
				if (keypad_timeout == KEYPAD_TIMEOUT)
				{
					keypad_timeout = 0;
					Read_Keypad();
				}
				key = Read_Pressed_Key();
				ret = Modify_Coord(coord_array, &real_coord, previous_coord, key);

				if ((ret == PRESSED_BACK_KEY) || (ret == PRESSED_HASH_KEY_TWO_TIME))
				{
					if (ret == PRESSED_HASH_KEY_TWO_TIME)
					{
						encoder_value = round((double)real_coord * ONE_ROTATION_TICK / ONE_ROTATION_VAL);
						Save_Coord(encoder_value);
						queue_var = REAL_COORD_CMD;
						xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
					}
					mode = MANUAL;
				}
			}
			mode_from = CALLIBRATION;
			break;

		case AUTO:
			if (keypad_timeout == KEYPAD_TIMEOUT)
			{
				keypad_timeout = 0;
				Read_Keypad();
			}
			key = Read_Pressed_Key();
			if (key == '*')
			{
				mode = MANUAL;
			}
			break;
	}

}

void send_messages_to_LCD(uint8_t system_mode, uint8_t *system_old_mode)
{
	if (*system_old_mode != system_mode)
	{
		if (system_mode != MANUAL)
		{
			queue_var = CLEAR_3_ROW;
			xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
		}
		if ((system_mode != MANUAL) && (system_mode != EDIT_SET_COORD) && (system_mode != CALLIBRATION))
		{
			queue_var = CURSOR_BLINKING_OFF;
			xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
		}

		switch(system_mode)
		{
			case MANUAL:
				if ((*system_old_mode == MENU) || (*system_old_mode == CALLIBRATION) || (*system_old_mode == AUTO)
						|| (*system_old_mode == EDIT_SET_COORD))
				{
					queue_var = REAL_CMD;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
					queue_var = CURRENT_REAL_COORD_CMD;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
					queue_var = CLEAR_2_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
					queue_var = CLEAR_3_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
				queue_var = MANUAL_MODE_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				queue_var = CURSOR_BLINKING_ON;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;

			case BRUSH_MOVING:
				if (*system_old_mode == EDIT_SET_COORD)
				{
					queue_var = CLEAR_2_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
					queue_var = CLEAR_3_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
				queue_var = BRUSH_MOVING_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;

			case HAND_CATCH:
				queue_var = HAND_CATCHING_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				if (*system_old_mode == CALLIBRATION)
				{
					queue_var = CLEAR_2_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
			break;

			case BRUSH_MOVE_WITH_BUTTONS:
				queue_var = BRUSH_MOVE_WITH_BUTTONS_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				if (*system_old_mode == CALLIBRATION)
				{
					queue_var = CLEAR_2_ROW;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
			break;

			case CALLIBRATION:
				if ((*system_old_mode == HAND_CATCH) || (*system_old_mode == BRUSH_MOVE_WITH_BUTTONS))
				{
					queue_var = REAL_COORD_CMD;
					xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				}
				queue_var = CALLIBRATION_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
				queue_var = CURSOR_BLINKING_ON;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;

			case MENU:
				queue_var = MENU_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;

			case EDIT_SET_COORD:
				queue_var = SET_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;

			case AUTO:
				queue_var = AUTO_CMD;
				xQueueSend(myQueue01Handle, (void* ) &queue_var, 10);
			break;
		}
		*system_old_mode = system_mode;
	}
}
