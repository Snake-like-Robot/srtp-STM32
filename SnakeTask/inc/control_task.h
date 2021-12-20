#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "stm32f4xx_hal.h"
#include "serpentine_task.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include "bluetooth_task.h"

#define N_BODY 7
#define N_HEAD 2
#define N (N_BODY+N_HEAD)
#define Pi 3.14159
/*---The length of these address are two.---*/
#define MODEL_NUMBER_ADD 0
#define CW_ANGLE_LIMIT_ADD 6
#define CCW_ANGLE_LIMIT_ADD 8
#define MAX_TORQUE_ADD 14
#define GOAL_POSITION_ADD 30
#define MOVING_SPEED_ADD 32
#define TORQUE_LIMIT_ADD 34
#define PRESENT_POSITION_ADD 36
#define PRESENT_SPEED_ADD 38
#define PRESENT_LOAD_ADD 40
#define PUNCH_ADD 48
/*---End of length of two---*/
#define TORQUE_ENABLE_ADD 24
#define LED_ENABLE_ADD 25

typedef enum
{
	serpentine,
	none
}snakeGait;
typedef enum
{
	Zero,
	Ping,
	Read,
	Write,
	Reg_Write,
	Action,
	Factory_Reset,
	Seven,
	Reboot,
	Sync_Write = 0x83,
	Bulk_Read = 0x92
}Instruction;

extern snakeGait gait;
extern int articularAngle[N];
extern double articularAngleRawValue[N];

void Control_Task_Init(void);
void Control_Task(void);
void Msg_Send_ArticularAngle_AX12A(void);
void Msg_Send_Single_AX12A(uint8_t packetId, uint8_t inst, uint8_t add, int data);
void my_strcat(char * str1, char * str2, int startAdd, int str2_len);
void SyncWriteMoving(uint8_t packetID,uint8_t inst,uint8_t add,int data[]);

#endif
