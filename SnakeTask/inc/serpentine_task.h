#ifndef SERPENTINE_TASK_H
#define SERPENTINE_TASK_H

#include "stm32f4xx_hal.h"
#include "control_task.h"
#include <math.h>
#include "bluetooth_task.h"

extern double serAlpha, serBeta, serGamma, serOmega;

void Serpentine_Init(void);
void Serpentine_Control(int t_ms);

#endif
