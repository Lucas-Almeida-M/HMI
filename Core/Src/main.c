/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *........................................
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ili9341.h"
//#include "fonts.h"
//#include "testimg.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "snow_tiger.h"
//#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "buttons.h"
#include "sensors.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "time.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern osMessageQueueId_t buttons_queueHandle;
extern osMessageQueueId_t sensors_queueHandle;
extern osMutexId_t sensor_mutexHandle;
extern osMutexId_t display_mutexHandle;
extern osTimerId_t buzzerTimerHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUZZ_ON       HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
#define BUZZ_OFF      HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
#define BUZZ_TOGGLE   HAL_GPIO_TogglePin(buzzer_GPIO_Port, buzzer_Pin);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcint[5] = {0};
uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;
Button button[MAX_BOTAO] = {0};
Navigation navigation = {0};
Sensors_Val sensorsVal = {0};
uint16_t uptime = 1;

Time currTime = {0};
uint64_t timeEpoch = 1727637780;
bool presenca = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  init_buttons();
  dc_meas_init();
  HAL_ADC_Start_DMA(&hadc1, adcint, 5);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim8);
   __HAL_TIM_SET_COUNTER(&htim3, 0);
   HAL_GPIO_WritePin(Rele0_GPIO_Port, Rele0_Pin, 1);
   HAL_GPIO_WritePin(Rele1_GPIO_Port, Rele1_Pin, 1);
   HAL_GPIO_WritePin(Rele2_GPIO_Port, Rele2_Pin, 1);
   HAL_GPIO_WritePin(Rele3_GPIO_Port, Rele3_Pin, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void epochToDateTime(uint64_t epochTime, Time *time)
{
    time_t rawtime = epochTime;
    struct tm *timeinfo;
    timeinfo = localtime(&rawtime);
    time->year = timeinfo->tm_year + 1900;
    time->month = timeinfo->tm_mon + 1;
    time->day= timeinfo->tm_mday;
    time->hour = timeinfo->tm_hour-3;
    time->minute = timeinfo->tm_min;
    time->second = timeinfo->tm_sec;
}

void show_time(void)
{
	char BufferText[50];
	epochToDateTime(timeEpoch, &currTime);
	sprintf(BufferText, "%02d/%02d/%d  %02d:%02d:%02d",
			currTime.day, currTime.month, currTime.year,
			currTime.hour, currTime.minute, currTime.second);
	ILI9341_DrawText(BufferText, FONT4, 10, 10, BLACK, WHITE);
}

int main_screen (void *args)
{
	char BufferText[50];

	show_time();

	if (navigation.pos.line == 0)
	{
		sprintf(BufferText, "> Visualizacao");
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Visualizacao");
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}

	if (navigation.pos.line == 1)
	{
		sprintf(BufferText, "> Comando");
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Comando");
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}

	if (navigation.pos.line == 2)
	{
		sprintf(BufferText, "> Configuracao");
		ILI9341_DrawText(BufferText, FONT4, 10, 70, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Configuracao");
		ILI9341_DrawText(BufferText, FONT4, 10, 70, BLACK, WHITE);
	}

	if (navigation.pos.line == 3)
	{
		sprintf(BufferText, "> Informacao");
		ILI9341_DrawText(BufferText, FONT4, 10, 90, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Informacao");
		ILI9341_DrawText(BufferText, FONT4, 10, 90, BLACK, WHITE);
	}
}

int visualization_screen (void *args)
{
	char BufferText[50];

	show_time();

	osMutexAcquire (sensor_mutexHandle, osWaitForever);
	for (int i = 0; i < 5; i++)
	{
		if (i == navigation.pos.line)
		{
			sprintf(BufferText, "> Sensor %d : %d,%d C       ", i, sensorsVal.adcVal[i] / 10, sensorsVal.adcVal[i] % 10);
		}
		else
		{
			sprintf(BufferText, "  Sensor %d : %d,%d C        ", i, sensorsVal.adcVal[i] / 10, sensorsVal.adcVal[i] % 10);
		}
		if ((sensorsVal.adcVal[i] / 10 )< navigation.screens[CONFIG_SCREEN].params[i])
			ILI9341_DrawText(BufferText, FONT4, 10, 30 + (i * 20), GREEN, WHITE);
		else
			ILI9341_DrawText(BufferText, FONT4, 10, 30 + (i * 20), BLUE, WHITE);
	}
	osMutexRelease (sensor_mutexHandle);
}


int command_screen (void *args)
{
	char BufferText[50];
	char* releStatus[2] = { " : Ligado   ", " : Desligado "};
	bool state[4] = {0};

	show_time();

	state[0] = HAL_GPIO_ReadPin(Rele0_GPIO_Port, Rele0_Pin);
	state[1] = HAL_GPIO_ReadPin(Rele1_GPIO_Port, Rele1_Pin);
	state[2] = HAL_GPIO_ReadPin(Rele2_GPIO_Port, Rele2_Pin);
	state[3] = HAL_GPIO_ReadPin(Rele3_GPIO_Port, Rele3_Pin);
	for (int i = 0; i < 4; i++)
	{
	    if (navigation.pos.line == i)
		{

			sprintf(BufferText, "> RELE %d%s", i, releStatus[state[i]]);
		}
		else
		{

			sprintf(BufferText, "  RELE %d%s", i, releStatus[state[i]]);
		}
	    ILI9341_DrawText(BufferText, FONT4, 10, 30 + i * 20, BLACK, WHITE);
	}
}
int configuration_screen (void *args)
{
	char BufferText[50];
	show_time();
	if (navigation.pos.line == 0)
	{
		sprintf(BufferText, "> Sensor 0 = %d", navigation.screens[navigation.actualScreen].params[0]);
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Sensor 0 = %d", navigation.screens[navigation.actualScreen].params[0]);
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}

	if (navigation.pos.line == 1)
	{
		sprintf(BufferText, "> Sensor 1 = %d", navigation.screens[navigation.actualScreen].params[1]);
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Sensor 1 = %d", navigation.screens[navigation.actualScreen].params[1]);
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}

	if (navigation.pos.line == 2)
	{
		sprintf(BufferText, "> Sensor 2 = %d", navigation.screens[navigation.actualScreen].params[2]);
		ILI9341_DrawText(BufferText, FONT4, 10, 70, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Sensor 2 = %d", navigation.screens[navigation.actualScreen].params[2]);
		ILI9341_DrawText(BufferText, FONT4, 10, 70, BLACK, WHITE);
	}

	if (navigation.pos.line == 3)
	{
		sprintf(BufferText, "> Sensor 3 = %d", navigation.screens[navigation.actualScreen].params[3]);
		ILI9341_DrawText(BufferText, FONT4, 10, 90, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Sensor 3 = %d", navigation.screens[navigation.actualScreen].params[3]);
		ILI9341_DrawText(BufferText, FONT4, 10, 90, BLACK, WHITE);
	}

	if (navigation.pos.line == 4)
	{
		sprintf(BufferText, "> Sensor 4 = %d", navigation.screens[navigation.actualScreen].params[4]);
		ILI9341_DrawText(BufferText, FONT4, 10, 110, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Sensor 4 = %d", navigation.screens[navigation.actualScreen].params[4]);
		ILI9341_DrawText(BufferText, FONT4, 10, 110, BLACK, WHITE);
	}

}
int information_screen (void *args)
{
	char BufferText[50];
	show_time();

	if (navigation.pos.line == 0)
	{
		sprintf(BufferText, "> Versao : 1.2.1");
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Versao : 1.2.1");
		ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);
	}

	if (navigation.pos.line == 1)
	{
		sprintf(BufferText, "> Uptime : %02d : %02d : %02d", uptime / 3600, (uptime % 3600) / 60 , uptime % 60);
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}
	else
	{
		sprintf(BufferText, "  Uptime : %02d : %02d : %02d", uptime / 3600, (uptime % 3600) / 60 , uptime % 60);
		ILI9341_DrawText(BufferText, FONT4, 10, 50, BLACK, WHITE);
	}
}


static void config_screen(uint16_t screenNum, int (*func_pointer)(void *args), uint8_t *redirect, bool habEdit, uint8_t lines_max)
{
	navigation.screens[screenNum].callback = func_pointer;
	navigation.screens[screenNum].habEdicao = habEdit;

	show_time();

	if (redirect[0] != 0)
	{
		for (int i = 0; i < 4; i++)
		{
			navigation.screens[screenNum].menuToRedirect[i] = redirect[i];
		}
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			navigation.screens[screenNum].menuToRedirect[i] = 0;
		}
	}

	if (habEdit)
	{
		navigation.screens[screenNum].params[0] = 28;
		navigation.screens[screenNum].params[1] = 28;
		navigation.screens[screenNum].params[2] = 28;
		navigation.screens[screenNum].params[3] = 50;
		navigation.screens[screenNum].params[4] = 70;
	}

	navigation.screens[screenNum].linesMax = lines_max - 1;
}

void init_screens(void)
{
	uint8_t main_redirect[4] = {VISUALIZATION_SCREEN, COMMAND_SCREEN, CONFIG_SCREEN, INFORMATION_SCREEN};

	config_screen(MAIN_SCREEN, main_screen, main_redirect, false, 4);
	config_screen(VISUALIZATION_SCREEN, visualization_screen, NULL,false, 5);
	config_screen(COMMAND_SCREEN, command_screen, NULL,false,4);
	config_screen(CONFIG_SCREEN, configuration_screen, NULL,true,5);
	config_screen(INFORMATION_SCREEN, information_screen, NULL,false,2);

	navigation.screens[navigation.actualScreen].callback(&navigation);
}



void led_set (uint8_t ledNum, bool state)
{
	switch (ledNum)
	{
		case LED0:
			HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, state);
			break;
		case LED1:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, state);
			break;
		case LED2:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, state);
			break;
		case LED3:
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state);
			break;
		case LED4:
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, state);
			break;
		case LED5:
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, state);
			break;
		case LED6:
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, state);
			break;
		case LED7:
			HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, state);
			break;
		case LED8:
			HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, state);
			break;
		case LED9:
			HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, state);
			break;
		case LED10:
			HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, state);
			break;
		case LED11:
			HAL_GPIO_WritePin(LD11_GPIO_Port, LD11_Pin, state);
			break;
	}
}

uint8_t identify_code(uint32_t code)
{
	uint8_t button = 0xff;
	switch (code)
	{
		case CODE_CIMA:
			button = BOTAO_CIMA;
		case CODE_BAIXO:
			button = BOTAO_BAIXO;
		case CODE_DIREITA:
			button = BOTAO_DIREITA;
		case CODE_ESQUERDA:
			button = BOTAO_ESQUERDA;
		case CODE_OK:
			button = BOTAO_OK;
		case CODE_ESC:
			button = BOTAO_ESC;
			break;
		default:

			break;
	}
}

uint8_t buttonEntry = 0xff;
bool debouncing = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case BT_AZUL_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_AZUL;
			break;
		case BT_CIMA_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_CIMA;
			break;
		case BT_BAIXO_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_BAIXO;
			break;
		case BT_DIREITA_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_DIREITA;
			break;
		case BT_ESQUERDA_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_ESQUERDA;
			break;
		case BT_OK_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_OK;
			break;
		case BT_ESC_Pin:
			if (debouncing == 0)
				buttonEntry = BOTAO_ESC;
			break;
		case Sensor_infravermelho_Pin:

			 if(GPIO_Pin == GPIO_PIN_11)
			  {
			    if (__HAL_TIM_GET_COUNTER(&htim3) > 8000)
			    {
			      tempCode = 0;
			      bitIndex = 0;
			    }
			    else if (__HAL_TIM_GET_COUNTER(&htim3) > 1700)
			    {
			      tempCode |= (1UL << (31-bitIndex));
			      bitIndex++;
			    }
			    else if (__HAL_TIM_GET_COUNTER(&htim3) > 1000)
			    {
			      tempCode &= ~(1UL << (31-bitIndex));
			      bitIndex++;
			    }
			    if(bitIndex == 32)
			    {
			      cmdli = ~tempCode;
			      cmd = tempCode >> 8;
			      if(cmdli == cmd)
			      {
			        code = tempCode;

			        buttonEntry = identify_code(code);
			      }
			    bitIndex = 0;
			    }
			  __HAL_TIM_SET_COUNTER(&htim3, 0);
			  }
			break;
		case Sensor_Presenca_Pin:
//			if (HAL_GPIO_ReadPin(Sensor_Presenca_GPIO_Port, Sensor_Presenca_Pin))
//				presenca = 1;
//			else
//				presenca = 0;
			break;
	}

	if (buttonEntry != 0xff)
	{
		debouncing = 1;
		__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
		HAL_TIM_Base_Start_IT(&htim5);
	}
}


void Display_TASK(void *argument)
{
	static char BufferText[40];
	ILI9341_FillScreen(WHITE);
	osDelay(500);
	ILI9341_SetRotation(SCREEN_VERTICAL_2);
	ILI9341_FillScreen(WHITE);
	HAL_TIM_Base_Start_IT(&htim2);
	init_screens();

  for(;;)
  {
	  if (HAL_GPIO_ReadPin(Sensor_Presenca_GPIO_Port, Sensor_Presenca_Pin))
	  {
		 HAL_GPIO_WritePin(backlight_GPIO_Port, backlight_Pin, 0);
	  }
	  else
	  {
		 HAL_GPIO_WritePin(backlight_GPIO_Port, backlight_Pin, 1);
		 osMutexAcquire (display_mutexHandle, osWaitForever);
		 navigation.screens[navigation.actualScreen].callback(&navigation);
		 osMutexRelease (display_mutexHandle);
	  }
     osDelay(500);
  }
}


int set_default_navigation_values(void)
{
  navigation.pos.page = PAGE_ZERO;
  navigation.pos.line = 0;
}

int callback_botao_esc (void *arguments)
{
	if (navigation.layerdepth == 0)
	{
		set_default_navigation_values();
		navigation.screens[navigation.actualScreen].callback(&navigation);
		return 0;
	}
	navigation.actualScreen = navigation.layers[--navigation.layerdepth];
	ILI9341_FillScreen(WHITE);
	set_default_navigation_values();
	navigation.screens[navigation.actualScreen].callback(&navigation);
}

int callback_botao_cima (void *arguments)
{
	if(navigation.pos.line == 0)
	{
		return 0;
	}
	navigation.pos.line--;
	navigation.screens[navigation.actualScreen].callback(&navigation);
}

int callback_botao_baixo (void *arguments)
{
	if(navigation.pos.line == navigation.screens[navigation.actualScreen].linesMax)
	{
		return 0;
	}
	navigation.pos.line++;
	navigation.screens[navigation.actualScreen].callback(&navigation);
}

int callback_botao_direita (void *arguments)
{
	if(!navigation.screens[navigation.actualScreen].habEdicao)
	{
		return 0;
	}

	if(navigation.screens[navigation.actualScreen].params[navigation.pos.line] >= 90)
	{
		return 0;
	}
	navigation.screens[navigation.actualScreen].params[navigation.pos.line]++;
	navigation.screens[navigation.actualScreen].callback(&navigation);

}

int callback_botao_esquerda (void *arguments)
{
	if(!navigation.screens[navigation.actualScreen].habEdicao)
	{
		return 0;
	}

	if(navigation.screens[navigation.actualScreen].params[navigation.pos.line] <= 15)
	{
		return 0;
	}

	navigation.screens[navigation.actualScreen].params[navigation.pos.line]--;
	navigation.screens[navigation.actualScreen].callback(&navigation);
}

int callback_botao_ok (void *arguments)
{
	if (navigation.actualScreen == COMMAND_SCREEN)
	{
		switch (navigation.pos.line)
		{
			case 0:
				if (HAL_GPIO_ReadPin(Rele0_GPIO_Port, Rele0_Pin))
					HAL_GPIO_WritePin(Rele0_GPIO_Port, Rele0_Pin, 0);
				else
					HAL_GPIO_WritePin(Rele0_GPIO_Port, Rele0_Pin, 1);
				break;
			case 1:
				if (HAL_GPIO_ReadPin(Rele1_GPIO_Port, Rele1_Pin))
					HAL_GPIO_WritePin(Rele1_GPIO_Port, Rele1_Pin, 0);
				else
					HAL_GPIO_WritePin(Rele1_GPIO_Port, Rele1_Pin, 1);
				break;
			case 2:
				if (HAL_GPIO_ReadPin(Rele2_GPIO_Port, Rele2_Pin))
					HAL_GPIO_WritePin(Rele2_GPIO_Port, Rele2_Pin, 0);
				else
					HAL_GPIO_WritePin(Rele2_GPIO_Port, Rele2_Pin, 1);
				break;
			case 3:
				if (HAL_GPIO_ReadPin(Rele3_GPIO_Port, Rele3_Pin))
					HAL_GPIO_WritePin(Rele3_GPIO_Port, Rele3_Pin, 0);
				else
					HAL_GPIO_WritePin(Rele3_GPIO_Port, Rele3_Pin, 1);
				break;
		}
		return 0;
	}
	if (navigation.screens[navigation.actualScreen].menuToRedirect[0])
	{
		navigation.actualScreen = navigation.screens[navigation.actualScreen].menuToRedirect[navigation.pos.line];
		navigation.layers[++navigation.layerdepth] = navigation.actualScreen;
		set_default_navigation_values();
		ILI9341_FillScreen(WHITE);
		navigation.screens[navigation.actualScreen].callback(&navigation);
	}
}

int callback_botao_azul (void *arguments)
{

}

void config_button (uint8_t buttonID, int (*funcPointer)(void *args))
{
	button[buttonID].callback = funcPointer;
}

void init_buttons(void)
{
	config_button(BOTAO_CIMA, callback_botao_cima);
	config_button(BOTAO_BAIXO, callback_botao_baixo);
	config_button(BOTAO_DIREITA, callback_botao_direita);
	config_button(BOTAO_ESQUERDA, callback_botao_esquerda);
	config_button(BOTAO_OK, callback_botao_ok);
	config_button(BOTAO_ESC, callback_botao_esc);
	config_button(BOTAO_AZUL, callback_botao_azul);
}

void button_buzzon(void)
{
	HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim4);
}

void Button_TASK(void *argument)
{
  uint8_t buttonID = 0;
  for(;;)
  {
	xQueueReceive(buttons_queueHandle, &buttonID, portMAX_DELAY);

	osMutexAcquire (display_mutexHandle, osWaitForever);

	button_buzzon();
	button[buttonID].callback(NULL);

	osMutexRelease (display_mutexHandle);
    osDelay(1);
  }
}


void sensor_task(void *argument)
{
  Sensors_Val sensors = {0};
  for(;;)
  {
	 if (xQueueReceive(sensors_queueHandle, &sensors, portMAX_DELAY))
	 {
		 osMutexAcquire (sensor_mutexHandle, osWaitForever);
		 memcpy(&sensorsVal, &sensors, sizeof(sensors));
		 osMutexRelease (sensor_mutexHandle);
	 }
    osDelay(1);
  }
}

void AlarmsTask(void *argument)
{
	Sensors_Val tempVal = {0};
	bool buzzerAlarm = 0;
  for(;;)
  {
	osMutexAcquire (sensor_mutexHandle, osWaitForever);
	memcpy(&tempVal, &sensorsVal, sizeof(Sensors_Val));
	osMutexRelease (sensor_mutexHandle);

	if((float)(tempVal.adcVal[0] / 10) < navigation.screens[CONFIG_SCREEN].params[0])
	{
		HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 1);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
	}
	else
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
		HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 0);
		buzzerAlarm = 1;
	}

	if((float)(tempVal.adcVal[1] / 10) < navigation.screens[CONFIG_SCREEN].params[1])
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	}
	else
	{
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
//		buzzerAlarm = 1;
	}

	if((float)(tempVal.adcVal[2] / 10) < navigation.screens[CONFIG_SCREEN].params[2])
	{
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 0);
	}
	else
	{
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 1);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0);
//		buzzerAlarm = 1;
	}

	if((float)(tempVal.adcVal[3] / 10) < navigation.screens[CONFIG_SCREEN].params[3])
	{
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);
		HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, 0);
	}
	else
	{
		HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, 1);
		HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0);
//		buzzerAlarm = 1;
	}

	if((float)(tempVal.adcVal[4] / 10) < navigation.screens[CONFIG_SCREEN].params[4])
	{
		HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, 1);
		HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 0);
	}
	else
	{
		HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 1);
		HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, 0);
//		buzzerAlarm = 1;
	}


	if (buzzerAlarm)
	{
		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
		osTimerStart(buzzerTimerHandle, 20);
		buzzerAlarm = 0;
	}
	osDelay(500);
  }
}

void buzzerTimer_calbk(void *argument)
{
	HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 0);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
  {
	  adc_update_sample();
  }
  if(htim->Instance == TIM4)
  {
	  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	  HAL_TIM_Base_Stop_IT(&htim4);
	  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 0);
  }

  if(htim->Instance == TIM5)
  {
	  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	  HAL_TIM_Base_Stop_IT(&htim5);
	  xQueueSendToBackFromISR(buttons_queueHandle, &buttonEntry, 1);
	  buttonEntry = 0xff;
	  debouncing = 0;
  }

  if(htim->Instance == TIM8)
  {
	  timeEpoch++;
	  uptime++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
