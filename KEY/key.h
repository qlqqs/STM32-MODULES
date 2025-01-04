#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/************************************* ����Ϊ�����궨�� *******************************************/
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/
/* �������Ŷ��� */
#define KEY_01_GPIO_PIN                   KEY_01_Pin
#define KEY_01_GPIO_PORT                  KEY_01_GPIO_Port
#define KEY_02_GPIO_PIN                   KEY_02_Pin
#define KEY_02_GPIO_PORT                  KEY_02_GPIO_Port
/******************* ��ֲ��Ҫ�޸ĵĺ궨�� ******************/

/* ����ʱ��������� */
#define KEY_DEBOUNCE_TIME    20      // ����ʱ�䣬��λms
#define KEY_SHORT_TIME       30      // �̰����ʱ�䣬��λms
#define KEY_LONG_TIME        1000    // ����ʱ����ֵ����λms
#define KEY_DOUBLE_TIME      300     // ˫�����ʱ�䣬��λms
#define KEY_LONG_REPEAT_TIME 500     // �����ظ������������λms

/* ����״̬���� */
#define KEY_RELEASED    1    // �����ͷ�
#define KEY_PRESSED     0    // ��������

/************************************** ����Ϊ�ṹ������ ******************************************/
/* ����ID���� */
typedef enum {
    KEY_01_ID = 0,
    KEY_02_ID,
    KEY_MAX_NUM
} key_id_t;

/* �����¼����� */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,    // �̰�
    KEY_EVENT_LONG_PRESS,     // ����
    KEY_EVENT_DOUBLE_CLICK,   // ˫��
    KEY_EVENT_RELEASE         // �ͷ�
} key_event_t;

/* ������Ϣ�ṹ�� */
typedef struct {
    uint8_t state;                   // ��ǰ״̬
    uint8_t last_state;              // ��һ��״̬
    uint32_t press_time;             // ����ʱ���
    uint32_t release_time;           // �ͷ�ʱ���
    uint32_t last_press;             // �ϴΰ���ʱ���(����˫�����)
    uint8_t press_count;             // ���´���(����˫�����)
    key_event_t event;               // �����¼�
    uint32_t last_long_press_time;   // �ϴγ�������ʱ��
} key_info_t;

/************************************** ����Ϊ�������� ******************************************/
/* �ⲿ�������� */
void key_init(void);
void key_exti_callback(uint16_t GPIO_Pin);
void key_update(void);
key_event_t key_get_event(key_id_t key_id);

#endif
