/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BT_AZUL_Pin GPIO_PIN_13
#define BT_AZUL_GPIO_Port GPIOC
#define BT_AZUL_EXTI_IRQn EXTI15_10_IRQn
#define BT_ESQUERDA_Pin GPIO_PIN_0
#define BT_ESQUERDA_GPIO_Port GPIOC
#define BT_ESQUERDA_EXTI_IRQn EXTI0_IRQn
#define buzzer_Pin GPIO_PIN_1
#define buzzer_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Sensor_infravermelho_Pin GPIO_PIN_6
#define Sensor_infravermelho_GPIO_Port GPIOA
#define Sensor_infravermelho_EXTI_IRQn EXTI9_5_IRQn
#define Rele0_Pin GPIO_PIN_4
#define Rele0_GPIO_Port GPIOC
#define Rele1_Pin GPIO_PIN_5
#define Rele1_GPIO_Port GPIOC
#define Rele2_Pin GPIO_PIN_0
#define Rele2_GPIO_Port GPIOB
#define Rele3_Pin GPIO_PIN_1
#define Rele3_GPIO_Port GPIOB
#define Sensor_Presenca_Pin GPIO_PIN_2
#define Sensor_Presenca_GPIO_Port GPIOB
#define Sensor_Presenca_EXTI_IRQn EXTI2_IRQn
#define BT_BAIXO_Pin GPIO_PIN_10
#define BT_BAIXO_GPIO_Port GPIOB
#define BT_BAIXO_EXTI_IRQn EXTI15_10_IRQn
#define BT_DIREITA_Pin GPIO_PIN_12
#define BT_DIREITA_GPIO_Port GPIOB
#define BT_DIREITA_EXTI_IRQn EXTI15_10_IRQn
#define LD11_Pin GPIO_PIN_13
#define LD11_GPIO_Port GPIOB
#define LD10_Pin GPIO_PIN_14
#define LD10_GPIO_Port GPIOB
#define LD9_Pin GPIO_PIN_15
#define LD9_GPIO_Port GPIOB
#define LD8_Pin GPIO_PIN_6
#define LD8_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_7
#define RST_GPIO_Port GPIOC
#define LD7_Pin GPIO_PIN_8
#define LD7_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_9
#define LD6_GPIO_Port GPIOC
#define BT_CIMA_Pin GPIO_PIN_8
#define BT_CIMA_GPIO_Port GPIOA
#define BT_CIMA_EXTI_IRQn EXTI9_5_IRQn
#define DC_Pin GPIO_PIN_9
#define DC_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_10
#define LD4_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_11
#define LD3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_12
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_15
#define LD1_GPIO_Port GPIOA
#define LD0_Pin GPIO_PIN_10
#define LD0_GPIO_Port GPIOC
#define backlight_Pin GPIO_PIN_11
#define backlight_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_12
#define LD5_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BT_OK_Pin GPIO_PIN_4
#define BT_OK_GPIO_Port GPIOB
#define BT_OK_EXTI_IRQn EXTI4_IRQn
#define BT_ESC_Pin GPIO_PIN_5
#define BT_ESC_GPIO_Port GPIOB
#define BT_ESC_EXTI_IRQn EXTI9_5_IRQn
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum
{
	LED0,
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	LED6,
	LED7,
	LED8,
	LED9,
	LED10,
	LED11
};

#define	CODE_CIMA  		1
#define	CODE_BAIXO    	2
#define	CODE_DIREITA	3
#define	CODE_ESQUERDA	4
#define	CODE_OK			5
#define	CODE_ESC		6


enum
{
	MAIN_SCREEN,
	VISUALIZATION_SCREEN,
	INFORMATION_SCREEN,
	COMMAND_SCREEN,
	CONFIG_SCREEN,
	MAX_SCREENS

};
typedef struct _Screen
{
	int (*callback)(void *args);
	bool habEdicao;
	uint16_t params[5];
	uint8_t menuToRedirect[4];
	uint8_t linesMax;
}Screen;

typedef struct _Position
{
	uint8_t page;
	uint8_t line;
}Position;

typedef struct _Navigation
{
	uint8_t layerdepth;
	uint8_t layers[2];
	Position pos;
	uint16_t actualScreen;
	Screen screens[MAX_SCREENS];
}Navigation;

typedef enum
{

	PAGE_ZERO,
	PAGE_ONE,
	PAGE_TWO,
	PAGE_THREE,
	PAGE_FOUR,
	PAGE_FIVE,
	PAGE_SIX

} pages;

enum
{
	BOTAO_CIMA,
	BOTAO_BAIXO,
	BOTAO_DIREITA,
	BOTAO_ESQUERDA,
	BOTAO_OK,
	BOTAO_ESC,
	BOTAO_AZUL,
	MAX_BOTAO
};

typedef struct Button_
{
	int (*callback)(void *args);
}Button;


typedef struct Time_
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
}Time;

void init_buttons(void);



void led_set (uint8_t ledNum, bool state);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
