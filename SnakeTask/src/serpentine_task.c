#include "serpentine_task.h"

double serAlpha, serBeta, serGamma, serOmega;
void Serpentine_Init(void)
{
	serAlpha = 20;
	serBeta = 60;
	serGamma = 0;
	serOmega = 3;
}
void Serpentine_Control(int t_ms)
{
	double t = t_ms * 1.0 / 1000;
	for(int i = 0; i < N_BODY-1; i++)
	{
		/*在没有映射前的关节角变化范围为-a+y——a+y*/
		articularAngleRawValue[i] = serAlpha*sin(serOmega*t+(i-1)*(serBeta/180*Pi))+serGamma;
	}
}
