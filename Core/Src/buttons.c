/*
 * buttons.c
 *
 *  Created on: Sep 16, 2024
 *      Author: lucas
 */

#include "buttons.h"
#include "FreeRTOS.h"
#include "queue.h"

Button button[MAX_BOTAO] = {0};
extern osMessageQueueId_t queue_buttonsHandle;


int callback_botao_cima (void *arguments)
{

}

int callback_botao_baixo (void *arguments)
{

}

int callback_botao_direita (void *arguments)
{

}

int callback_botao_esquerda (void *arguments)
{

}

int callback_botao_ok (void *arguments)
{

}

int callback_botao_azul (void *arguments)
{

}


static void config_button (uint8_t buttonID, int (*funcPointer)(void *args))
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
	config_button(BOTAO_AZUL, callback_botao_azul);
}


void Button_TASK(void *argument)
{
	uint8_t buttonID = 0;
  for(;;)
  {
	xQueueReceive(queue_buttonsHandle, &buttonID, portMAX_DELAY);

	button[buttonID].callback(NULL);
    osDelay(1);
  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t button = 0xff;
	switch (GPIO_Pin)
	{
		case GPIO_PIN_13:
			button = BOTAO_AZUL;
			break;
		case GPIO_PIN_8:
			button = BOTAO_CIMA;
			break;
		case GPIO_PIN_10:
			button = BOTAO_BAIXO;
			break;
		case GPIO_PIN_4:
			button = BOTAO_DIREITA;
			break;
		case GPIO_PIN_5:
			button = BOTAO_ESQUERDA;
			break;
		case GPIO_PIN_0:
			button = BOTAO_OK;
			break;
	}

	if (button != 0xff)
	{
		xQueueSendToBack(queue_buttonsHandle, &button, 1);
	}
}
