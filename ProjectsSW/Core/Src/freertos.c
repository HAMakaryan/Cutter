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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern char coord_array[COORD_SIZE];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 1024 * 4
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
		  if (val == MAIN_MENU_CMD)
		  {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) " *-Edit #-Cut C-Cal ");
			//Write_LCD_Buffer((char*) " *-Edit #-Cut C-Cal ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == REAL_COORD_CMD) {
			LCD_SendCommand(LCD_ADDR, R_COORD_POS);
			Write_LCD_Buffer(coord_array, COORD_SIZE, R_COORD_POS);

		  } else if (val == SET_COORD_CMD) {
			LCD_SendCommand(LCD_ADDR, S_COORD_POS);
			Write_LCD_Buffer(coord_array, COORD_SIZE, S_COORD_POS);

		  } else if (val == SPACE_FOR_MAX_MIN_1_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0x8E);
			LCD_SendString(LCD_ADDR, (char*) "   ");
			//Write_LCD_Buffer((char*)"   ", 3, 0x8E);

		  } else if (val == SPACE_FOR_MAX_MIN_2_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0xCE);
			LCD_SendString(LCD_ADDR, (char*) "   ");
			//Write_LCD_Buffer((char*)"   ", 3, 0xCE);

		  } else if (val == EDIT_MODE_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "     Edit Mode      ");
			//Write_LCD_Buffer((char*)"     Edit Mode      ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == MIN_2_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0xCE);
			LCD_SendString(LCD_ADDR, (char*) "Min");
			//Write_LCD_Buffer((char*)"Min", 3, 0xCE);

		  } else if (val == MAX_2_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0xCE);
			LCD_SendString(LCD_ADDR, (char*) "Max");
			//Write_LCD_Buffer((char*)"Max", 3, 0xCE);

		  } else if (val == ARE_YOU_SURE_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "   Are you sure?    ");
			//Write_LCD_Buffer((char*)"   Are you sure?    ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == MIN_1_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0x8E);
			LCD_SendString(LCD_ADDR, (char*) "Min");
			//Write_LCD_Buffer((char*)"Min", 3, 0x8E);

		  } else if (val == MAX_1_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, 0x8E);
			LCD_SendString(LCD_ADDR, (char*) "Max");
			//Write_LCD_Buffer((char*)"Max", 3, 0x8E);

		  } else if (val == ZERO_S_COORD_CMD) {
			LCD_SendCommand(LCD_ADDR, S_COORD_POS);
			//Write_LCD_Buffer((char*)"00000", COORD_SIZE, S_COORD_POS);
			LCD_SendString(LCD_ADDR, "0000.0");

		  } else if (val == SPACE_4_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "                    ");
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == ZERO_R_COORD_CMD) {
			LCD_SendCommand(LCD_ADDR, R_COORD_POS);
			//Write_LCD_Buffer((char*)"00000", COORD_SIZE, R_COORD_POS);
			Write_LCD_Buffer((char*)"00000", COORD_SIZE, R_COORD_POS);

		  } else if (val == SPACE_2_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_2);
			LCD_SendString(LCD_ADDR, (char*) "                    ");
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_2);

		  } else if (val == SPACE_3_ROW_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_3);
			LCD_SendString(LCD_ADDR, (char*) "                    ");
			//Write_LCD_Buffer((char*)"                    ", LCD_ROW_SIZE, ROW_3);

		  } else if (val == CALLIBRATION_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_3);
			LCD_SendString(LCD_ADDR, (char*) "    Callibration    ");
			//Write_LCD_Buffer((char*)"    Callibration    ", LCD_ROW_SIZE, ROW_3);

		  } else if (val == CUTTING_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "      Cutting       ");
			//Write_LCD_Buffer((char*)"      Cutting       ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == CUT_IS_DONE_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "    Cut is done     ");
			//Write_LCD_Buffer((char*)"    Cut is done     ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == ALLOWED_CUTTING_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "  Allowed cutting   ");
			//Write_LCD_Buffer((char*)"  Allowed cutting   ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == HAND_CATCHING_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "   Hand catching    ");
			//Write_LCD_Buffer((char*)"   Hand catching    ", LCD_ROW_SIZE, ROW_4);

		  } else if (val == REAL_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_1);
			LCD_SendString(LCD_ADDR, (char*) "Real  ");
			//Write_LCD_Buffer((char*)"Real  ", sizeof("Real  "), ROW_1);

		  } else if (val == SET_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_2);
			LCD_SendString(LCD_ADDR, (char*) "Set   ");
			//Write_LCD_Buffer((char*)"Set   ", sizeof("Set   "), ROW_2);

		  } else if (val == BRUSH_MOVING_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_4);
			LCD_SendString(LCD_ADDR, (char*) "    Brush Moving    ");
			//LCD_SendString(LCD_ADDR, "    Brush Moving    ");

		  } else if (val == ENCODER_VAL_CMD) {
			LCD_SendCommand(LCD_ADDR, ROW_3);
			Write_LCD_Buffer(temp_buf_enc, 7, ROW_3);
			//LCD_SendString(LCD_ADDR, "    Brush Moving    ");
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
