/*
 * buttons.h
 *
 *  Created on: Sep 16, 2024
 *      Author: lucas
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_


#include "cmsis_os2.h"
#include "main.h"

enum
{
	BOTAO_CIMA,
	BOTAO_BAIXO,
	BOTAO_DIREITA,
	BOTAO_ESQUERDA,
	BOTAO_OK,
	BOTAO_AZUL,
	MAX_BOTAO
};

typedef struct Button_
{
	int (*callback)(void *args);
}Button;


void init_buttons(void);

#endif /* INC_BUTTONS_H_ */
