/**
  * @author lzh
  * @note 串口：USART2,波特率：115200,蓝牙模块名称：BLUE
  * @protocol 	帧头：0xff 0xff
                命令：	0x00 停止
						0x01 前进
							0x** 前进速度（1—5）
						0x02 左转（定速）
							0x** 转弯角度（0—90）
						0x03 右转（定速）
							0x** 转弯角度（0—90）
						0x04 后退
							0x** 后退速度（1—5）
						0x05 左后（定速）
							0x** 转弯角度（0—90）
						0x06 右后（定速）
							0x** 转弯角度（0—90）
						0x07 头部高度
							0x** 抬起高度（从0到最高映射为0—255）
						0x08 头部转动
							0x00 固定角度
								0x**角度大小（±45°映射为0—255）
							0x01 自稳定模式
						0x09 单电机调试
							0x** ID
								0x** ** 角度大小（大字端,0—1023）
							
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
