/*
 * cutter.h
 *
 *  Created on: May 19, 2020
 *      Author: AnnaV
 */

#ifndef INC_CUTTER_H_
#define INC_CUTTER_H_

#include "stm32f7xx_hal.h"

//#define TEST

#define LCD_ROW_SIZE	20 //20x4
#define LCD_BUF_SIZE	100
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
#define R_COORD_POS			0x81
#define S_COORD_POS			0xD5
#define REAL				1
#define	SET					2

#define TIMEOUT_PRINT_REAL			1000
#define TIMEOUT_FOR_WAIT_ABCD		60000

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


//popoxelu gorcakicner

//#define DEBUG_MODE

//#define FORWARD_COEFFICIENT 0.4		//EXTRA_COORD -i 0.4 mas@ sharjvum e mid aragutyamb(FORWARD(aysinqn tesoghakan dashtic hervanum e sanr@))

//#define BACK_COEFFICIENT_1 0.2		//(encoder_value - set_tick) taracutyan 0.2 mas@ sharjvum e max aragutyamb(BACK(tesoghakan dashtin motenum e sanr@))
//#define BACK_COEFFICIENT_2 0.2		//(encoder_value - set_tick) taracutyan 0.2 mas@ sharjvum e mid aragutyamb(BACK(tesoghakan dashtin motenum e sanr@))


#define TIMEOUT_TO_ACTIVATE_CUTTING_BUTTON 	3000 // /4000 er
#define TIMEOUT_TO_CUT 						500  // /1000 er

#define	ONE_ROTATION_VAL	(double)11.985	//mm		//11.962 mek ptuyti arjeq@ milimetrov
#define ONE_ROTATION_TICK	1000						//mek ptuyti depqum tickeri tiv@

#define MAX_SPEED	2420
#define MID_SPEED	1600
#define MIN_SPEED   1390	//1410 er

#define EXTRA_COORD 	50		//lracucich tick araj gnalu hamar(tesoghakan dashtic sanr@ hervanum e)

#define DELTA			5		//chshtelu gorcakic

#define SOFT_LIMIT_UP				1050	//mm
#define HARD_LIMIT_UP				1056	//mm
#define LIMIT_DOWN					95		//mm

#define HARD_LIMIT_UP_IN_TICK		(uint32_t)round(((double)HARD_LIMIT_UP * ONE_ROTATION_TICK / ONE_ROTATION_VAL))
#define LIMIT_DOWN_IN_TICK			(uint32_t)round(((double)LIMIT_DOWN * ONE_ROTATION_TICK / ONE_ROTATION_VAL))

//#define MIN_DISTANCE_IN_TICK		1100		//minimum taracutyun@, voric cacri depqum minimum aragutyamb e sharjvelu

#define BACK_FORWARD_BUTTON_PRESSED_TIME 2000	//2second

typedef enum {
	CALLIBRATION,
	BRUSH_MOVE,
	HAND_CATCH,
	CUTTING,
	MANUAL,
	ENTER_SET_COORD,
	BRUSH_MOVING,
	EDIT_SET_COORD,
	BRUSH_MOVE_WITH_BUTTONS,
	AUTO,
	EDIT_SET_COORD_AUTO,
	WAIT_FOR_START,
	START,
	MENU,
	WAIT_FOR_CUTTING
}system_mode_t;

typedef enum {
	CALLIBRATION_CMD,
	CUTTING_CMD,

	REAL_CMD,
	SET_CMD,
	GO_TO_CMD,
	SAVE_CMD,
	REAL_COORD_CMD,
	SET_COORD_CMD,
	MANUAL_MODE_CMD,
	CURRENT_REAL_COORD_CMD,

	CUT_IS_DONE_CMD,
	ALLOWED_CUTTING_CMD,
	HAND_CATCHING_CMD,
	BRUSH_MOVING_CMD,
	BRUSH_MOVE_WITH_BUTTONS_CMD,
	CURSOR_BLINKING_OFF,
	CURSOR_BLINKING_ON,

	MENU_CMD,
	CLEAR_ROW1_CMD,
	CLEAR_2_ROW,
	CLEAR_3_ROW,

	AUTO_CMD,
	ABCD_CMD,
	A_CMD,
	B_CMD,
	C_CMD,
	D_CMD,
	EDIT_SET_COORD_CMD,
	WAIT_FOR_START_CMD,
	START_CMD,
	WAIT_FOR_CUTTING_CMD,
	COORD_ERROR_CMD,
	BRUSH_MOVE_WITH_BUTTONS_CMD_AUTO,
	HAND_CATCHING_CMD_AUTO,
	MIN_COORD_CMD,
	MAX_COORD_CMD,
	MIN_COORD_CAL_CMD,
	MAX_COORD_CAL_CMD,
	MIN_MAX_DEL_CMD,
	SAVE_AND_EXIT_CMD,
	ENTER_THE_CORRECT_COORDINATE_CMD

} lcd_commands;

typedef enum {

	BRUSH_MOVING_ENDED = 1,
	PRESSED_HASH_KEY_TWO_TIME,
	REAL_SET_COORDS_DIFF_LOW,
	PRESSED_BACK_KEY,
	COORD_IS_NOT_CHANGED,
	PEDAL_RELEASED,
	HAND_CATCH_RELEASED,
	FORWARD_BACK_BUTTONS_RELEASED,
	GO_BACK,
	PRESSED_ABCD,
	COORD_IS_NOT_ZERO,

} Ret_Values;

void LCD_Init(uint8_t lcd_addr);
void LCD_Write(uint8_t lcd_addr);
void Collect_Digits(void);
void Keypad_Init(void);
void Save_Coord(uint32_t coord);
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
void Print_Coord(double r_coord, uint8_t coord_name);
void Print_Current_Coord();
void state_machine(void);
void Main_Task(void);
void LCD_Write_Coord(char* buf);
void send_messages_to_LCD(uint8_t system_mode, uint8_t *system_old_mode);
void set_cursor_for_ABCD(void);
void Read_Coords(void);

char Convert_Key_to_Char(uint8_t key);
uint8_t Get_Coord_Size(char* coord_arr, double coord);
uint8_t Read_Pedal(void);
uint8_t Read_Knife_Sensors(void);
uint8_t Read_Hand_Catch_Input(void);
uint16_t Read_Encoder(void);
uint32_t Read_Coord(void);
double Create_Number(char* buf);

typedef struct
{
	uint8_t cut_cnt_for_st0;
	uint8_t pedal_cnt_for_st0;
	uint8_t hand_catch_cnt_for_st0;
	uint8_t forward_cnt_for_st0;
	uint8_t back_cnt_for_st0;

	uint8_t cut_cnt_for_st1;
	uint8_t pedal_cnt_for_st1;
	uint8_t hand_catch_cnt_for_st1;
	uint8_t forward_cnt_for_st1;
	uint8_t back_cnt_for_st1;

	uint8_t cut_is_pressed;
	uint8_t pedal_is_pressed;
	uint8_t hand_catch_is_pressed;
	uint8_t forward_is_pressed;
	uint8_t back_is_pressed;

} Input_State;

#endif /* INC_CUTTER_H_ */
