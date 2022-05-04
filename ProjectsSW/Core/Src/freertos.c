/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "cutter.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern char temp_buf_enc[7];
extern char current_coord[6];
extern int32_t encoder_value;
extern uint16_t speed;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern char coord_array[COORD_SIZE];
extern double real_coord;
extern double set_coord;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (1, sizeof(uint8_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
	  //osDelay(10000);
	  Main_Task();
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t val = 0;
	  if(xQueueReceive(myQueue01Handle, &val, 0) == pdTRUE )
	  {
		  switch(val)
		  {
			case CURSOR_BLINKING_OFF:
				LCD_SendCommand(LCD_ADDR,0x0C);
			break;

			case CURSOR_BLINKING_ON:
				LCD_SendCommand(LCD_ADDR,0x0F);
			break;

			case SET_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " #-Save  ");
				LCD_SendCommand(LCD_ADDR,ROW_3+3);
				LCD_SendString(LCD_ADDR, " *-Del");
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S             B-Back");
			break;

			case SET_COORD_CMD:
				LCD_SendCommand(LCD_ADDR, S_COORD_POS);
				Write_LCD_Buffer(coord_array, COORD_SIZE, 0xDA);
				LCD_SendCommand(LCD_ADDR,ROW_4);
			break;

			case CURRENT_REAL_COORD_CMD:
				LCD_SendCommand(LCD_ADDR, R_COORD_POS);
				double temp_real_coord = real_coord;
				char temp_coord[7];
				sprintf(temp_coord, "%6.1f", temp_real_coord);
				LCD_SendString(LCD_ADDR, temp_coord);
			break;

			case REAL_COORD_CMD:
				LCD_SendCommand(LCD_ADDR, R_COORD_POS);
				Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);
				LCD_SendCommand(LCD_ADDR,ROW_1);
			break;

			case REAL_CMD:
				LCD_SendCommand(LCD_ADDR, ROW_1);
				LCD_SendString(LCD_ADDR, "R                   ");
			break;

			case MANUAL_MODE_CMD:
				LCD_SendCommand(LCD_ADDR, ROW_1+14);
				LCD_SendString(LCD_ADDR, "Manual");
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S      *-Auto D-Menu");
				LCD_SendCommand(LCD_ADDR,ROW_4+1);
			break;

			case BRUSH_MOVING_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S       Brush Moving");
			break;

			case HAND_CATCHING_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S      Hand Catching");
			break;

			case BRUSH_MOVE_WITH_BUTTONS_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S       Brush Moving");
			break;

			case CALLIBRATION_CMD:
				LCD_SendCommand(LCD_ADDR, ROW_1+14);
				LCD_SendString(LCD_ADDR, "Callib");
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " #-Save  ");
				LCD_SendCommand(LCD_ADDR,ROW_3+3);
				LCD_SendString(LCD_ADDR, " *-Del");
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S             B-Back");
				LCD_SendCommand(LCD_ADDR,ROW_1);
			break;

			case MENU_CMD:
				LCD_SendCommand(LCD_ADDR, ROW_1+14);
				LCD_SendString(LCD_ADDR, "  Menu");
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " C-Callib");
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S             B-Back");
			break;

			case CUTTING_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S            Cutting");
			break;

			case CUT_IS_DONE_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S        Cut is done");
			break;

			case ALLOWED_CUTTING_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S    Allowed cutting");
			break;

			case AUTO_CMD:
				LCD_SendCommand(LCD_ADDR, ROW_1+13);
				LCD_SendString(LCD_ADDR, "A      ");
				LCD_SendCommand(LCD_ADDR,ROW_2+13);
				LCD_SendString(LCD_ADDR, "B      ");
				LCD_SendCommand(LCD_ADDR,ROW_3+13);
				LCD_SendString(LCD_ADDR, "C      ");
				LCD_SendCommand(LCD_ADDR,ROW_4);
				LCD_SendString(LCD_ADDR, "S                   ");
				LCD_SendCommand(LCD_ADDR,ROW_4+13);
				LCD_SendString(LCD_ADDR, "D      ");

				LCD_SendCommand(LCD_ADDR, ROW_1+8);
				LCD_SendString(LCD_ADDR, "Auto");
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " #-Start");
				LCD_SendCommand(LCD_ADDR,ROW_3+3);
				LCD_SendString(LCD_ADDR, " *-Manual");
			break;

			case GO_TO_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " #-Go to ");
			break;

			case SAVE_CMD:
				LCD_SendCommand(LCD_ADDR,ROW_2+3);
				LCD_SendString(LCD_ADDR, " #-Save  ");
			break;

			case CLEAR_2_ROW:
				LCD_SendCommand(LCD_ADDR, ROW_2);
				LCD_SendString(LCD_ADDR, "                    ");
			break;

			case CLEAR_3_ROW:
				LCD_SendCommand(LCD_ADDR, ROW_3);
				LCD_SendString(LCD_ADDR, "                    ");
			break;
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
