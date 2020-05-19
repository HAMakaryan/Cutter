/*
 * cutter.h
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#ifndef INC_CUTTER_H_
#define INC_CUTTER_H_

#include "stm32f7xx_hal.h"

void LCD_Init(void);
void LCD_Write(uint8_t* string, uint8_t row, uint8_t col, uint8_t length);
void Keypad_Init(void);
void Save_Coord(uint32_t address, float coord);
void Set_Inverter(uint8_t dir, uint16_t speed);
void Change_Speed(uint16_t speed);
void Brush_Brake_Off(void);
void Brush_Brake_On(void);
void Move_Brush(void);
void Solid_On();
void Solid_Off();
void Pressing_On(void);
void Pressing_Off(void);
void Cutting_On(void);
void Cutting_Off(void);

uint8_t Read_Keypad(void);
uint8_t Read_Pedal(void);
uint8_t Read_Knife_Sensors(void);
uint8_t Read_Hand_Catch_Input(void);

uint16_t Read_Encoder(void);

float Read_Coord(void);

#endif /* INC_CUTTER_H_ */
