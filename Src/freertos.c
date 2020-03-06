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
#include "u8g2.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCK_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_1
#define MOSI_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_14
#define LCD_RESET_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
u8g2_t _u8g2;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for canCommandsQueue */
osMessageQueueId_t canCommandsQueueHandle;
const osMessageQueueAttr_t canCommandsQueue_attributes = {
  .name = "canCommandsQueue"
};
/* Definitions for canSendTimer */
osTimerId_t canSendTimerHandle;
const osTimerAttr_t canSendTimer_attributes = {
  .name = "canSendTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  //Initialize SPI peripheral
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    /* HAL initialization contains all what we need so we can skip this part. */
    break;

  //Function which implements a delay, arg_int contains the amount of ms
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;

  //Function which delays 10us
  case U8X8_MSG_DELAY_10MICRO:
    for (uint16_t n = 0; n <= 280; n++)
    {
      __NOP();
    }
    break;

  case U8X8_MSG_DELAY_NANO:
    for (uint16_t n = 0; n <= (arg_int / 36); n++)
    {
      __NOP();
    }

  //Function which delays 100ns
  case U8X8_MSG_DELAY_100NANO:
    for (uint16_t n = 0; n <= 3; n++)
    {
      __NOP();
    }
    break;

  // Function to define the logic level of the CS line
  case U8X8_MSG_GPIO_CS:
    if (arg_int)
      HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
    else
      HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);

    break;

  //Function to define the logic level of the RESET line
  case U8X8_MSG_GPIO_RESET:
    if (arg_int)
      HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
    else
      HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
    break;

  default:
    return 0; //A message was received which is not implemented, return 0 to indicate an error
  }

  return 1; // command processed successfully.
}

uint8_t u8x8_byte_3wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    HAL_SPI_Transmit(&hspi2, (uint8_t *)arg_ptr, arg_int, HAL_MAX_DELAY);
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
    u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
    break;
  default:
    return 0;
  }
  return 1;
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void canSendTimerCallback(void *argument);

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

  /* Create the timer(s) */
  /* creation of canSendTimer */
  canSendTimerHandle = osTimerNew(canSendTimerCallback, osTimerPeriodic, NULL, &canSendTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // osTimerStart(canSendTimerHandle, 1000);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canCommandsQueue */
  canCommandsQueueHandle = osMessageQueueNew (8, sizeof(uint16_t), &canCommandsQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  // uint32_t current_time = osKernelGetTickCount();
  u8g2_Setup_st7920_s_128x64_1(&_u8g2, U8G2_R0, u8x8_byte_3wire_hw_spi, u8g2_gpio_and_delay_stm32);
  u8g2_InitDisplay(&_u8g2);     // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&_u8g2, 0); // wake up display

  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
  HAL_Delay(200);

  /* Infinite loop */
  for (;;)
  {
    // uint32_t current_time = osKernelGetTickCount();
    u8g2_ClearDisplay(&_u8g2);
    u8g2_FirstPage(&_u8g2);
    do
    {
      /* all graphics commands have to appear within the loop body. */
      u8g2_SetFont(&_u8g2, u8g2_font_ncenB14_tr);
      u8g2_DrawStr(&_u8g2, 0, 20, "Cam + Yudi!");
    } while (u8g2_NextPage(&_u8g2));
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* canSendTimerCallback function */
void canSendTimerCallback(void *argument)
{
  /* USER CODE BEGIN canSendTimerCallback */

  /* USER CODE END canSendTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
