/**
  * @author lzh
  * @note uart���ͣ�uart6
		  usb���ͣ�Ĭ�ϵ�usb�ڣ�Ҳֻ��һ���ڣ�
		  �������ĵ�����TIM9��ɣ��޸Ķ�ʱ����������Ҫ��CUMEMX�н����޸�
		  �����е�articularAngle���ؽڽǣ�Ĭ��ʹ�ýǶ���
          �����IDĬ��Ϊ��0��ʼ������β��ʼΪ0
          �����˶�ָ��ĵ��ֻ��Ҫ��1�ŵ����ʼ
  * @problem 	usb�����д�����
				8/27�����������������
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
	for(int j = 1; j <= 5; j++)//ѭ��5�Σ�ȷ���ɹ�
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
		/*�����������ݵĴ���������serpentine*/
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
		/*���͵�������*/
		HAL_UART_Transmit_IT(&huart6, instructionPacketSingle, 7 + singleLength);
		bluetoothCommand = lastBluetoothCommand;
	}
}
/*��������ķ���Ƶ��ȡ���ڶ�ʱ��������*/
void Msg_Send_ArticularAngle_AX12A(void)
{
	memset(instructionPacket,0,100);
	instructionPacketLength = 0;
	/*-------------������----------------*/
//	for(int i = 0; i < N_BODY-1; i++)
//		articularAngle[i] = 512;
	/*-------------������----------------*/
//	for(int i = 1; i < 3; i++)
//		Msg_Send_Single_AX12A(i, Write, GOAL_POSITION_ADD, articularAngle[i]);
	SyncWriteMoving(0xfe, Sync_Write, GOAL_POSITION_ADD, articularAngle);
	HAL_UART_Transmit_IT(&huart6, instructionPacket, instructionPacketLength);
}
void Msg_Send_Single_AX12A(uint8_t packetId, uint8_t inst, uint8_t add, int data)
{
	/*�������ݰ��Ĵ���*/
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
	/*һ�������ݡ������ݰ�����*/
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
		/*�淶�Ƕȵ�����Ϊ0����360��*/
		while(temp<=-180) temp += 360;
		while(temp>180) temp -= 360;
		/*���ؽڽ�ת��Ϊ����ռ�ĽǶ�*/
		temp += 150;
		/*�Ƕ�ӳ��*/
		temp = temp / 300 * 1023;
		/*��е��λΪ280��720����һ��ת�����䣨312��712�������������Ӧ��alpha��30��ʵ�ʵ�alpha������Ϊ�����������С��*/
		temp *= 1.955;
		temp -= 488;
		/*��һ����ͷ�����ǣ�����ȶ��ĽǶȳ�����е��λ����������λ����*/
		if(temp > 720) temp = 720;
		if(temp < 280) temp = 280;
		articularAngle[i] = (int)temp;
	}
	for(int i = N_BODY; i < N; i++)
	{
		temp = articularAngleRawValue[i];
		/*�淶�Ƕȵ�����Ϊ0����360��*/
		while(temp<=-180) temp += 360;
		while(temp>180) temp -= 360;
		/*���ؽڽ�ת��Ϊ����ռ�ĽǶ�*/
		temp += 150;
		/*�Ƕ�ӳ��*/
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
/*	packetID	�����ţ�����Ҫ�����ж���������ã���дΪ0*FE
	inst		����˵��������������д����   д����Ϊ0x83
	add			�Ĵ����׵�ַ 
	data		�������飬����������0��3��4��ֵ��������Զ��0��3��4������
	L			�����ֽڳ��ȣ�����goal position�������ֽڳ���Ϊ2
	
	N			��������+1 
	N_BODY      �������
	N_HEAD		ͷ������
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
