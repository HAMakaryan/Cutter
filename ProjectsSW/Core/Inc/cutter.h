/*
 * cutter.h
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#ifndef INC_CUTTER_H_
#define INC_CUTTER_H_

#include "stm32f7xx_hal.h"

#define LCD_BUF_SIZE	32
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

#define IDLE	0
#define ERROR 	1
#define SINGLE	2

#define EDIT 		0
#define BRUSH_MOVE	1
#define CUTTING		2

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

void LCD_Init(uint8_t lcd_addr);
void LCD_Write(uint8_t lcd_addr);
void Create_Number(void);
void Keypad_Init(void);
void Save_Coord(float coord);
void Set_Inverter(uint8_t dir, uint16_t speed);
void Change_Speed(uint16_t speed);
void Brush_Brake_Off(void);
void Brush_Brake_On(void);
void Move_Brush(void);
void Solid_On(void);
void Solid_Off(void);
void Pressing_On(void);
void Pressing_Off(void);
void Cutting_On(void);
void Cutting_Off(void);
void Read_Keypad(void);
void Keypad_Write_Buffer(uint8_t data);
void LCD_Write_Buffer(uint16_t *data, uint8_t size);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendString(uint8_t lcd_addr, char *str);

uint8_t Read_Pedal(void);
uint8_t Read_Knife_Sensors(void);
uint8_t Read_Hand_Catch_Input(void);
uint8_t Convert_Key_to_Char(uint8_t key);
uint16_t Read_Encoder(void);
uint16_t Read_Coord(void);

void Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t * st0_counter,
					uint8_t * st1_counter, uint8_t * is_pressed, uint8_t on_state);
void Read_Inputs(void);


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
