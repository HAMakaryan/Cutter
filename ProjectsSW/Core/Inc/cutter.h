/*
 * cutter.h
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#ifndef INC_CUTTER_H_
#define INC_CUTTER_H_

#include "stm32f7xx_hal.h"

#define LCD_ROW_SIZE	20 //20x4
#define LCD_BUF_SIZE	64
#define LCD_ADDR		(0x27 << 1)
#define LCD_DELAY_MS	5
#define LCD_TIMEOUT		10
#define LCD_DATA_MASK	0x0100

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define ROW_SIZE			4
#define COL_SIZE			4
#define KEYPAD_BUF_SIZE		16
#define DEBOUNCE_TIME		10
#define KEYPAD_TIMEOUT		5 //every 5 ms
#define INPUT_TIMEOUT		5 //every 5 ms
#define ROW_1				0x80
#define ROW_2				0xC0
#define ROW_3				0x94
#define ROW_4				0xD4
#define R_COORD_POS			0x86
#define S_COORD_POS			0xC6
#define REAL				1
#define	SET					2

#define TIMEOUT_TO_ACTIVATE_CUTTING_BUTTON 	5000
#define TIMEOUT_TO_CUT 						3000

#define IDLE	0
#define ERROR 	1
#define SINGLE	2

#define COORD_SIZE				5
#define COORD_SIZE_WITH_POINT	COORD_SIZE+1

#define PRESSED		1
#define RELEASED	0
#define SINGLE_KEY	1

#define ROW1	1
#define ROW2	2
#define ROW3	4
#define ROW4	8
#define COL1	1
#define COL2	2
#define COL3	4
#define COL4	8

#define FORWARD	1
#define	BACK	2
#define STOP	3

#define RAMP_UP		1
#define RAMP_DOWN	2

#define RAMP_UP_VAL		20
#define RAMP_DOWN_VAL	20

#define	MAX_DAC_VALUE				4095
#define SOFT_LIMIT_UP				1050
#define HARD_LIMIT_UP				1056	//mm
#define LIMIT_DOWN					95		//mm
#define MIN_DISTANCE				8333.3	//100mm
#define MIN_SPEED					400
#define ENC_VAL_FOR_RAMP_DOWN		416.7		//5mm
#define INTERVAL_FOR_RAMP			4000	//4 second
#define TIME_FOR_CHANGE_RAMP		10

#define TIMEOUT_PRINT_REAL			1000

typedef enum {
	SELECT,
	EDIT,
	CALLIBRATION,
	BRUSH_MOVE,
	HAND_CATCH,
	CHECK_PEDAL,
	CUTTING
}system_mode_t;

void LCD_Init(uint8_t lcd_addr);
void LCD_Write(uint8_t lcd_addr);
void Collect_Digits(void);
void Keypad_Init(void);
void Save_Coord(float coord);
void Set_Inverter(uint8_t dir, uint16_t speed);
void Change_Speed(uint16_t* speed, uint8_t ramp);
void Brush_Unlock(void);
void Brush_Lock(void);
void Move_Brush(void);
void Cutting_On(void);
void Cutting_Off(void);
void Read_Keypad(void);
void Keypad_Write_Buffer(char data);
void LCD_Write_Buffer(uint16_t *data, uint8_t size);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendString(uint8_t lcd_addr, char *str);
void Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t * st0_counter,
					uint8_t * st1_counter, uint8_t * is_pressed, uint8_t on_state);
void Read_Inputs(void);
void Print_Coord(float r_coord, uint8_t coord_name);
void state_machine(void);
void Main_Task(void);

char Convert_Key_to_Char(uint8_t key);
uint8_t Get_Coord_Size(char* coord_arr, float coord);
uint8_t Read_Pedal(void);
uint8_t Read_Knife_Sensors(void);
uint8_t Read_Hand_Catch_Input(void);
uint16_t Read_Encoder(void);
uint32_t Read_Coord(void);
float Create_Number(char* buf);

typedef struct
{
	uint8_t cut_cnt_for_st0;
	uint8_t pedal_cnt_for_st0;
	uint8_t hand_catch_cnt_for_st0;

	uint8_t cut_cnt_for_st1;
	uint8_t pedal_cnt_for_st1;
	uint8_t hand_catch_cnt_for_st1;

	uint8_t cut_is_pressed;
	uint8_t pedal_is_pressed;
	uint8_t hand_catch_is_pressed;

} Input_State;

#endif /* INC_CUTTER_H_ */
