/**
  * @author lzh
  * @note ���ڣ�USART2,�����ʣ�115200,����ģ�����ƣ�BLUE
  * @protocol 	֡ͷ��0xff 0xff
                ���	0x00 ֹͣ
						0x01 ǰ��
							0x** ǰ���ٶȣ�1��5��
						0x02 ��ת�����٣�
							0x** ת��Ƕȣ�0��90��
						0x03 ��ת�����٣�
							0x** ת��Ƕȣ�0��90��
						0x04 ����
							0x** �����ٶȣ�1��5��
						0x05 ��󣨶��٣�
							0x** ת��Ƕȣ�0��90��
						0x06 �Һ󣨶��٣�
							0x** ת��Ƕȣ�0��90��
						0x07 ͷ���߶�
							0x** ̧��߶ȣ���0�����ӳ��Ϊ0��255��
						0x08 ͷ��ת��
							0x00 �̶��Ƕ�
								0x**�Ƕȴ�С����45��ӳ��Ϊ0��255��
							0x01 ���ȶ�ģʽ
						0x09 ���������
							0x** ID
								0x** ** �Ƕȴ�С�����ֶ�,0��1023��
							
  */
#include "bluetooth_task.h"

void bluetoothClear(void);

int bluetoothRxState;
int bluetoothRxcount = 1;
RC_Command bluetoothCommand = Stop;
RC_Command lastBluetoothCommand;
RotationType headRotationType = Auto;
int singleModeID = 0;
int commandData = 0;

void bluetoothReceive(uint8_t data)
{
	if(data == 0xff && bluetoothRxState == 0)
	{
		bluetoothRxState++;
	}
	else if(data == 0xff && bluetoothRxState == 1)
	{
		bluetoothRxState++;
	}
	else if(bluetoothRxState == 2)
	{
		switch(data)
		{
			case Stop:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = Stop;
				bluetoothRxState = 0;
				break;
			case Forward:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = Forward;
				bluetoothRxState++;
				break;
			case Left:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = Left;
				bluetoothRxState++;
				break;
			case Right:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = Right;
				bluetoothRxState++;
				break;
			case Back:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = Back;
				bluetoothRxState++;
				break;
			case BackLeft:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = BackLeft;
				bluetoothRxState++;
				break;
			case BackRight:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = BackRight;
				bluetoothRxState++;
				break;
			case HeadHeight:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = HeadHeight;
				bluetoothRxState++;
				break;
			case HeadRotation:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = HeadRotation;
				bluetoothRxState++;
				break;
			case SingleMode:
				lastBluetoothCommand = bluetoothCommand;
				bluetoothCommand = SingleMode;
				bluetoothRxState++;
				break;
			default:
				bluetoothClear();
				break;
		}
	}
	else if(bluetoothRxState == 3)
	{
		if(bluetoothCommand == HeadRotation)
		{
			switch(data)
			{
				case Auto:
					headRotationType = Auto;
					bluetoothRxState = 0;
					break;
				case Manual:
					headRotationType = Manual;
					bluetoothRxState++;
					break;
				default:
					bluetoothClear();
					break;
			}
		}
		else if(bluetoothCommand == SingleMode)
		{
			singleModeID = data;
			bluetoothRxState++;
		}
		else
		{
			commandData = data;
			bluetoothRxState = 0;
		}
	}
	else if(bluetoothRxState == 4)
	{
		if(bluetoothCommand == HeadRotation && headRotationType == Manual)
		{
			commandData = data;
			bluetoothRxState = 0;
		}
		else if(bluetoothCommand == SingleMode)
		{
			if(bluetoothRxcount == 1)
			{
				commandData = data;
				commandData = commandData << 8;
				bluetoothRxcount++;
			}
			else if(bluetoothRxcount == 2)
			{
				commandData += data;
				bluetoothRxcount = 1;
				bluetoothRxState = 0;
			}
			else
			{
				bluetoothClear();
			}
		}
	}
	else
	{
		bluetoothClear();
	}
}
void bluetoothClear(void)
{
	bluetoothRxState = 0;
	bluetoothRxcount = 1;
	bluetoothCommand = Stop;
	headRotationType = Auto;
	singleModeID = -1;
	commandData = -1;
}
