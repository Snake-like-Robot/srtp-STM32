#ifndef BLUETOOTH_TASK_H
#define BLUETOOTH_TASK_H

#include "stm32f4xx_hal.h"
#include "control_task.h"

typedef enum
{
	Stop,
	Forward,
	Left,
	Right,
	Back,
	BackLeft,
	BackRight,
	HeadHeight,
	HeadRotation,
	SingleMode
}RC_Command;

typedef enum
{
	Manual,
	Auto
}RotationType;

extern RC_Command bluetoothCommand;
extern RC_Command lastBluetoothCommand;
extern RotationType headRotationType;
extern int singleModeID;
extern int commandData;

void bluetoothReceive(uint8_t data);

#endif
