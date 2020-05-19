/*
 * cutter.c
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#include "stm32f7xx_hal.h"
#include <stdint.h>


/**
  * @brief	Initializes the LCD(16x4) module using I2C peripheral.
  * @param	None
  * @retval	None
  */
void LCD_Init()
{
	/* Initializes the following
	* Function Set
	* Entry mode
	* Display on/off
	* Clear Display
	* Needed to declare LCD_Write_Command() function
	  to send command to the LCD
	  */
}

/**
  * @brief	Displays a string on the LCD.
  * @param	The string to be displayed
  * @param	The length of the string
  * @param	The LCD row
  * @param	The LCD column
  * @retval None
  */
void LCD_Write(uint8_t* string, uint8_t length, uint8_t row, uint8_t col)
{
	/* Needed to declare Set_Cursor() and LCD_Write_Char()
	 *to implement this function */
}

/**
  * @brief	Initializes the keypad(4x4).
  * @param	None
  * @retval None
  */
void Keypad_Init()
{

}

/**
  * @brief	Reads keypad data.
  * @param	None
  * @retval Can be 0-15 (pressed key code) or NO_PRESSED.
  */
uint8_t Read_Keypad()
{
	/* This function scans the keypad and returns the status of button
	 * If any button is pressed this function returns the key code
	 * If none of the buttons is pressed returns NO_PRESSED
	 * */
	return 0;
}

/**
  * @brief	Saves the brush coordinate to the Backup register.
  * @param	The address of the Backup register
  * @param	The value to be saved
  * @retval None
  */
void Save_Coord(uint32_t address, float coord)
{
	/* To write to the Backup register user needs to
	 * Unlock the Backup register to access it
	 * Write value to the Backup Register
	 * Lock the backup register
	 */
}

/**
  * @brief	Reads the value of the Backup register.
  * @param	Address of the backup register
  * @retval The real coordinate of the brush
  */
float Read_Coord(uint32_t address)
{
	/* Reads real coordinate of the brush */
	return 0;
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


