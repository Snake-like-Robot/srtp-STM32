/**
  * @author lzh
  * @note uart发送：uart6
		  usb发送：默认的usb口（也只有一个口）
		  本函数的调用由TIM9完成，修改定时器的周期需要在CUMEMX中进行修改
		  程序中的articularAngle（关节角）默认使用角度制
          电机的ID默认为从0开始，从蛇尾开始为0
          发送运动指令的电机只需要从1号电机开始
  * @problem 	usb发送有待测试
				8/27：单电机调试有问题
  */

#include "control_task.h"

void ArticularAngleProcessing(void);
void HeadStability(void);
double my_abs(double data);

snakeGait gait = serpentine;
Instruction instruction;
int t_ms = 0;
int articularAngle[N] = {0};
double articularAngleRawValue[N] = {0};
uint8_t instructionPacketSingle[100] = {0};
uint8_t instructionPacket[1000] = {0};

static uint8_t singleLength = 0;
static int instructionPacketLength = 0;
static int isAddLengthTwo[49] = {0};
static int tag[20] = {
11,
MODEL_NUMBER_ADD,
CW_ANGLE_LIMIT_ADD,
CCW_ANGLE_LIMIT_ADD,
MAX_TORQUE_ADD,
GOAL_POSITION_ADD,
MOVING_SPEED_ADD,
TORQUE_LIMIT_ADD,
PRESENT_POSITION_ADD,
PRESENT_SPEED_ADD,
PRESENT_LOAD_ADD,
PUNCH_ADD};

void Control_Task_Init(void)
{
	for(int i = 1;i<=tag[0];i++)
		isAddLengthTwo[tag[i]]=1;
	Serpentine_Init();
	for(int j = 1; j <= 5; j++)//循环5次，确保成功
	{
		memset(instructionPacket,0,100);
		instructionPacketLength = 0;
		for(int i = 0; i < N; i++)
			Msg_Send_Single_AX12A(i, Write, TORQUE_ENABLE_ADD, 1);
		HAL_UART_Transmit_IT(&huart6, instructionPacket, instructionPacketLength);
		HAL_Delay(100);
		memset(instructionPacket,0,100);
		instructionPacketLength = 0;
		for(int i = 0; i < N; i++)
			Msg_Send_Single_AX12A(i, Write, LED_ENABLE_ADD, 1);
		HAL_UART_Transmit_IT(&huart6, instructionPacket, instructionPacketLength);
		HAL_Delay(100);
	}
	HAL_TIM_Base_Start_IT(&htim9);
}

void Control_Task(void)
{
	if(bluetoothCommand != Stop && bluetoothCommand != SingleMode)
	{
		/*蓝牙接收数据的处理，适用于serpentine*/
		if(bluetoothCommand == Forward)
		{
			serOmega = commandData;
			serAlpha = my_abs(serAlpha);
			serGamma = 0;
		}
		if(bluetoothCommand == Back)
		{
			serOmega = -commandData;
			serAlpha = -my_abs(serAlpha);
			serGamma = 0;
		}
		if(bluetoothCommand == Left)
		{
			serAlpha = my_abs(serAlpha);
			serOmega = 2;
			serGamma = -commandData/N_BODY;
		}
		if(bluetoothCommand == Right)
		{
			serAlpha = my_abs(serAlpha);
			serOmega = 2;
			serGamma = commandData/N_BODY;
		}
		if(bluetoothCommand == BackLeft)
		{
			serAlpha = -my_abs(serAlpha);
			serOmega = -2;
			serGamma = commandData/N_BODY;
		}
		if(bluetoothCommand == BackRight)
		{
			serAlpha = -my_abs(serAlpha);
			serOmega = -2;
			serGamma = -commandData/N_BODY;
		}
		if(bluetoothCommand == HeadHeight)
		{
			articularAngleRawValue[N_BODY] = commandData*0.3529;
			articularAngleRawValue[N_BODY+1] = -articularAngleRawValue[N_BODY];
		}
		t_ms+=Get_TIM_Period_ms(9);
		switch(gait)
		{
			case serpentine:
				Serpentine_Control(t_ms);
				break;
			case none:
				break;
			default:
				break;
		}
		if(bluetoothCommand == HeadRotation)
		{
			if(headRotationType == Auto)
				HeadStability();
			else if(headRotationType == Manual)
			{
				articularAngleRawValue[N_BODY-1] = commandData*0.3529 - 45;
			}
		}
		ArticularAngleProcessing();
		Msg_Send_ArticularAngle_AX12A();
	}
	else if(bluetoothCommand == SingleMode)
	{
		articularAngle[singleModeID] = commandData;
		Msg_Send_Single_AX12A(singleModeID, Write, GOAL_POSITION_ADD, articularAngle[singleModeID]);
		/*发送单个数据*/
		HAL_UART_Transmit_IT(&huart6, instructionPacketSingle, 7 + singleLength);
		bluetoothCommand = lastBluetoothCommand;
	}
}
/*这个函数的发送频率取决于定时器的周期*/
void Msg_Send_ArticularAngle_AX12A(void)
{
	memset(instructionPacket,0,100);
	instructionPacketLength = 0;
	/*-------------调试用----------------*/
//	for(int i = 0; i < N_BODY-1; i++)
//		articularAngle[i] = 512;
	/*-------------调试用----------------*/
//	for(int i = 1; i < 3; i++)
//		Msg_Send_Single_AX12A(i, Write, GOAL_POSITION_ADD, articularAngle[i]);
	SyncWriteMoving(0xfe, Sync_Write, GOAL_POSITION_ADD, articularAngle);
	HAL_UART_Transmit_IT(&huart6, instructionPacket, instructionPacketLength);
}
void Msg_Send_Single_AX12A(uint8_t packetId, uint8_t inst, uint8_t add, int data)
{
	/*发送数据包的处理*/
	singleLength = 0;
	int check = 0;
	memset(instructionPacketSingle,0,100);
	instructionPacketSingle[0] = 0xff;
	instructionPacketSingle[1] = 0xff;
	instructionPacketSingle[2] = packetId;
	if(isAddLengthTwo[add] == 1)
		singleLength = 2;
	else
		singleLength = 1;
	instructionPacketSingle[3] = singleLength + 3;
	instructionPacketSingle[4] = inst;
	instructionPacketSingle[5] = add;
	for(int index = 6; index < 6 + singleLength; index++)
	{
		instructionPacketSingle[index] = data & 0xff;
		data = data >> 8;
	}
	for(int i = 2; i < 6 + singleLength; i++)
		check += instructionPacketSingle[i];
	check = ~check;
	instructionPacketSingle[6 + singleLength] = check & 0xff;
	/*一起发送数据——数据包整合*/
	my_strcat((char *)instructionPacket, (char *)instructionPacketSingle, instructionPacketLength, 7+singleLength);
	instructionPacketLength+=7 + singleLength;
}
void my_strcat(char * str1, char * str2, int startAdd, int str2_len)
{
	for(int i = 0; i < str2_len; i++)
		*(str1+startAdd+i) = *(str2+i);
}
double my_abs(double data)
{
	if(data <= 0)
		return -data;
	else
		return data;
}
void ArticularAngleProcessing(void)
{
	double temp;
	for(int i = 0; i < N_BODY; i++)
	{
		temp = articularAngleRawValue[i];
		/*规范角度的区间为0——360°*/
		while(temp<=-180) temp += 360;
		while(temp>180) temp -= 360;
		/*将关节角转化为舵机空间的角度*/
		temp += 150;
		/*角度映射*/
		temp = temp / 300 * 1023;
		/*机械限位为280—720，这一步转换区间（312—712）（这个参数对应的alpha是30，实际的alpha可能因为缩减振幅而变小）*/
		temp *= 1.955;
		temp -= 488;
		/*这一步给头部考虑，如果稳定的角度超过机械限位，就锁到限位即可*/
		if(temp > 720) temp = 720;
		if(temp < 280) temp = 280;
		articularAngle[i] = (int)temp;
	}
	for(int i = N_BODY; i < N; i++)
	{
		temp = articularAngleRawValue[i];
		/*规范角度的区间为0——360°*/
		while(temp<=-180) temp += 360;
		while(temp>180) temp -= 360;
		/*将关节角转化为舵机空间的角度*/
		temp += 150;
		/*角度映射*/
		temp = temp / 300 * 1023;
		articularAngle[i] = (int)temp;
	}
}
void HeadStability(void)
{
	double headAngle = 0;
	for(int i = 0; i < N_BODY-1; i++)
	{
		for(int j = 0; j <= i; j++)
		{
			headAngle += articularAngleRawValue[j];
		}
	}
	headAngle /= N_BODY;
	articularAngleRawValue[N_BODY-1] = headAngle;
}
void SyncWriteMoving(uint8_t packetID,uint8_t inst,uint8_t add,int data[])
{
/*	packetID	舵机编号，若需要对所有舵机产生作用，则写为0*FE
	inst		命令说明，例如读入命令，写命令   写命令为0x83
	add			寄存器首地址 
	data		数据数组，例如数组中0，3，4有值，则表明对舵机0，3，4有命令
	L			数据字节长度，例如goal position的数据字节长度为2
	
	N			最大舵机编号+1 
	N_BODY      身体节数
	N_HEAD		头部节数
*/
	int temp;
	uint8_t L;
  	if(isAddLengthTwo[add] == 1)
		L = 2;
	else
		L = 1;
	instructionPacket[0]=instructionPacket[1]=0xff;
	instructionPacket[2]=packetID;
	instructionPacket[3]=(L+1)*N+4;
	instructionPacket[4]=inst;
	instructionPacket[5]=add;
	instructionPacket[6]=L;
	int i,tag=7,j;
	for (i=0;i<N;i++)
		if (data[i])
			{
				temp = data[i];
				instructionPacket[tag++]=i;
				for (j=1;j<=L;j++){
					instructionPacket[tag++]=temp&(0xff);
					temp>>=8;
				}
			}
	for (i=2;i<tag;i++) instructionPacket[tag]+=instructionPacket[i];
	instructionPacket[tag]=~instructionPacket[tag];
	instructionPacketLength=tag+1;
}
