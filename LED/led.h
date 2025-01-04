#ifndef _LED_H
#define _LED_H

#include "main.h"

/************************************** ����Ϊ�����궨�� ******************************************/
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/
/* ���� ���� */
/* LED1���Ŷ��� */
#define LED_01_GPIO_PIN                 LED_01_Pin        /* LED1���ź� */
#define LED_01_GPIO_PORT                LED_01_GPIO_Port  /* LED1�˿� */

/* LED2���Ŷ��� */
#define LED_02_GPIO_PIN                 LED_02_Pin        /* LED2���ź� */
#define LED_02_GPIO_PORT                LED_02_GPIO_Port  /* LED2�˿� */

/* LED3���Ŷ��� */
#define LED_03_GPIO_PIN                 LED_03_Pin        /* LED3���ź� */
#define LED_03_GPIO_PORT                LED_03_GPIO_Port  /* LED3�˿� */
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/

/************************************** ����Ϊ�������� ******************************************/
/* �ⲿ�ӿں���*/
void LED_01_ON(void);                                     /* LED1�� */
void LED_01_OFF(void);                                    /* LED1�ر� */
void LED_01_TOGGLE(void);                                 /* LED1��ת */

void LED_02_ON(void);                                     /* LED2�� */
void LED_02_OFF(void);                                    /* LED2�ر� */
void LED_02_TOGGLE(void);                                 /* LED2��ת */

void LED_03_ON(void);                                     /* LED3�� */
void LED_03_OFF(void);                                    /* LED3�ر� */
void LED_03_TOGGLE(void);                                 /* LED3��ת */

#endif
