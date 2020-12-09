/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <driver/WPB_Mani_driver.h>
#include <math.h>

static bool bFirstQuart = true;

CWPB_Mani_driver::CWPB_Mani_driver()
{
   	m_SendBuf = new unsigned char[1024];
	memset(m_SendBuf, 0, 1024);
	memset(m_ParseBuf, 0, 128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 14;

	for (int i = 0; i < 4; i++)
	{
		arValAD[i] = 0;
	}
	for (int i = 0; i < 4; i++)
	{
		arMotorCurrent[i] = 0;
		arMotorPos[i] = 0;
	}
	for (int i = 0; i < 5; i++)
	{
		arManiSpeed[i] = 2000;
		arManiPos[i] = 2048;
		arManiCurrentRecv[i] = 0;
		arManiPosRecv[i] = 0;
	}
	arManiSpeed[1] = 6000;
	arManiSpeed[3] = 1200;
	arManiSpeed[4] = 5000;
	fVoltage = 0;
	nParseCount = 0;
	fQuatW = 0;
	fQuatX = 0;
	fQuatY = 0;
	fQuatZ = 0;
	
	fGyroX = 0;
	fGyroY = 0;
	fGyroZ = 0;
	
	fAccX = 0;
	fAccY = 0;
	fAccZ = 0;

	fCurYaw = 0;
	fFirstYaw = 0;
	bCalFirstYaw = false; 

	fLinearAccLimit = 0.2;
	fAngularAccLimit = 0.1;

	//mani gripper
	arManiGripperValue[0] = 0;
	arManiGripperPos[0] = 47998;
	arManiGripperValue[1] = 0.034;
	arManiGripperPos[1] = 40000;
	arManiGripperValue[2] = 0.07;
	arManiGripperPos[2] = 30000;
	arManiGripperValue[3] = 0.102;
	arManiGripperPos[3] = 20000;
	arManiGripperValue[4] = 0.133;
	arManiGripperPos[4] = 10000;
	arManiGripperValue[5] = 0.16;
	arManiGripperPos[5] = 0;

	nLastCmdLiftPos = 0;
	nLastCmdGripperPos = 0;
}
    
CWPB_Mani_driver::~CWPB_Mani_driver()
{
	delete []m_SendBuf;
}


void CWPB_Mani_driver::Parse(unsigned char inData)
{
	m_ParseBuf[m_nRecvIndex] = inData;

	if (m_lastRecv == 0x55 && inData == 0xAA && m_bFrameStart == 0)
	{
		m_bFrameStart = 1;
		m_ParseBuf[0] = m_lastRecv;
		m_ParseBuf[1] = inData;
		m_nRecvIndex = 2;
		m_lastRecv = 0x00;
		return;
	}

	if (m_bFrameStart)
	{
		if (m_nRecvIndex == 3)
		{
			m_nFrameLength = inData + 8;
		}

		//put received data into buffer
		m_ParseBuf[m_nRecvIndex] = inData;
		m_nRecvIndex++;

		//receive one frame, invoke ParseFrame to parse
		if (m_nRecvIndex == m_nFrameLength)
		{
			m_DisRecv();
			m_ParseFrame();
			m_bFrameStart = false;
		}

		//receive buffer overflow
		if (m_nRecvIndex >= 128)
		{
			//m_ResetRcvBuf();
			m_bFrameStart = 0;
		}
	}
	else
		m_lastRecv = inData;
}


void CWPB_Mani_driver::m_Split2Bytes(unsigned char *inTarg, short inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned short temp;
	memcpy(&temp, &inSrc, sizeof(short));
	inTarg[1] = (unsigned char)temp & 0x00ff;

	temp >>= 8;

	inTarg[0] = (unsigned char)temp & 0x00ff;
}


void CWPB_Mani_driver::m_Split4Bytes(unsigned char *inTarg, int inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned int temp;
	memcpy(&temp, &inSrc, sizeof(int));
	inTarg[3] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[2] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[1] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[0] = (unsigned char)temp & 0x00ff;
}


short CWPB_Mani_driver::m_WordFromChar(unsigned char *inBuf)
{
	static short wtemp;
	wtemp = 0;
	wtemp = *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}

int CWPB_Mani_driver::m_IntFromChar(unsigned char *inBuf)
{
	static int itemp;
	itemp = 0;
	itemp = *(inBuf);

	itemp <<= 8;
	itemp |= *(inBuf + 1);

	itemp <<= 8;
	itemp |= *(inBuf + 2);

	itemp <<= 8;
	itemp |= *(inBuf + 3);

	return itemp;
}


void CWPB_Mani_driver::m_CalSendSum(unsigned char* pNewCmdBuf)
{
	int nLen = pNewCmdBuf[3] + 7;

	pNewCmdBuf[nLen - 1] = 0x00;
	for (int i = 0; i < nLen - 1; i++)
	{
		pNewCmdBuf[nLen - 1] += pNewCmdBuf[i];
	}
}


void CWPB_Mani_driver::m_ParseFrame()
{
	nParseCount = 0;
	if (m_ParseBuf[4] == 0x01)	//ϵͳ��ѹ
	{
		short wValue = m_WordFromChar(&m_ParseBuf[7]);
		fVoltage = (float)wValue * 0.01;
	}
	if (m_ParseBuf[4] == 0x06)	//IO
	{
		for (int i = 0; i < 4; i++)
		{
			unsigned char tmp = 0x01;
			tmp = tmp<<i;
			tmp = tmp & m_ParseBuf[8];
			if (tmp == 0)
			{
				arValIOInput[i] = 0;
			} 
			else
			{
				arValIOInput[i] = 1;
			}
		}
	}

	if (m_ParseBuf[4] == 0x07)	//AD
	{
		//AD 0~4
		for (int i = 0; i < 4; i++)
		{
			arValAD[i] = m_WordFromChar(&m_ParseBuf[7 + i * 2]);
		}
	}

	if (m_ParseBuf[4] == 0x08)	//电机模块
	{
		if (m_ParseBuf[5] == 0x60)	//底盘电机
		{
			int nCurMotorID = m_ParseBuf[7] - 1;
			if (nCurMotorID < 4)
			{
				arMotorCurrent[nCurMotorID] = m_IntFromChar(&m_ParseBuf[8]);
				arMotorPos[nCurMotorID] = m_IntFromChar(&m_ParseBuf[12]);
			}
			else
			{
				//id超限
			}
		}
		if (m_ParseBuf[5] == 0x63)	//机械臂关节模块
		{
			int nServoID = m_ParseBuf[7];
			if (nServoID >= 11 && nServoID <= 15)
			{
				int nIndex = nServoID - 11;
				arManiCurrentRecv[nIndex] = m_IntFromChar(&m_ParseBuf[8]);
				arManiPosRecv[nIndex] = m_IntFromChar(&m_ParseBuf[12]);
			}
			//printf("Pos = %d    cur = %d\n",arManiPosRecv[15],arManiCurrentRecv[15]);
		}
	}

	if (m_ParseBuf[4] == 0x09)	//IMU
	{
		if(m_ParseBuf[6] == 0x01)	//GYRO
		{
			fGyroX = (float)m_Piece2int(&m_ParseBuf[7]);
			fGyroY = (float)m_Piece2int(&m_ParseBuf[11]);
			fGyroZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x02)	//ACC
		{
			fAccX = (float)m_Piece2int(&m_ParseBuf[7]);
			fAccY = (float)m_Piece2int(&m_ParseBuf[11]);
			fAccZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x03)	//QUAT-W-X
		{
			fQuatW = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatX = (float)m_Piece2int(&m_ParseBuf[11]);
		}
		if(m_ParseBuf[6] == 0x04)	//QUAT-Y-Z
		{
			fQuatY = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatZ = (float)m_Piece2int(&m_ParseBuf[11]);
			// yaw: (about Z axis)
    		//fCurYaw = atan2(2*fQuatX*fQuatY - 2*fQuatW*fQuatZ, 2*fQuatW*fQuatW + 2*fQuatX*fQuatX - 1);
			//printf("[CWPB_Home_driver] fYaw = %.2f\n",fCurYaw);
			if(bFirstQuart == true)
			{
				//fFirstYaw = fCurYaw;
				bCalFirstYaw = true;
				bFirstQuart = false;
			}
		}
	}
}


void CWPB_Mani_driver::m_DisRecv()
{
	
}



int CWPB_Mani_driver::GenCmd(int inBuffOffset, int inDevID, int inModule, int inMethod, unsigned char* inData, int inDataLen)
{
	int nCmdLen = 0;

	int nTailIndex = inBuffOffset + 7 + inDataLen;
	if (nTailIndex >= 1024)
	{
		return nCmdLen;
	}

	unsigned char* pNewCmd = m_SendBuf + inBuffOffset;
	pNewCmd[0] = 0x55;
	pNewCmd[1] = 0xaa;
	pNewCmd[2] = (unsigned char)inDevID;
	pNewCmd[3] = (unsigned char)inDataLen;
	pNewCmd[4] = (unsigned char)inModule;
	pNewCmd[5] = (unsigned char)inMethod;
	memcpy(&pNewCmd[6], inData, inDataLen);

	m_CalSendSum(pNewCmd);

	nCmdLen = inDataLen + 7;
	return nCmdLen;
}


void CWPB_Mani_driver::SendMotors(int inMotor1, int inMotor2, int inMotor3, int inMotor4)
{
	static unsigned char arMotorSpeedData[16];
	m_Split4Bytes(arMotorSpeedData, inMotor1);
	m_Split4Bytes(arMotorSpeedData + 4, inMotor2);
	m_Split4Bytes(arMotorSpeedData + 8, inMotor3);
	m_Split4Bytes(arMotorSpeedData + 12, inMotor4);
	int nCmdLenght = GenCmd(0, 0x41, 0x08, 0x60, arMotorSpeedData, 16);
	Send(m_SendBuf, nCmdLenght);
}


void CWPB_Mani_driver::SendLED(int inMode,  unsigned char inR,  unsigned char inG,  unsigned char inB)
{
	unsigned char arLEDData[4];
	arLEDData[0] = inMode;
	arLEDData[1] = inR;
	arLEDData[2] = inG;
	arLEDData[3] = inB;
	int nCmdLenght = GenCmd(0, 0x41, 0x0a, 0x70, arLEDData, 4);
	Send(m_SendBuf, nCmdLenght);
}


void CWPB_Mani_driver::SendOutput(int inOut1, int inOut0)
{
	unsigned char arOutputData = 0;
	if (inOut1 > 0)
	{
		arOutputData = arOutputData | 0x02;
	}
	if (inOut0 > 0)
	{
		arOutputData = arOutputData | 0x01;
	}
	int nCmdLenght = GenCmd(0, 0x41, 0x0a, 0x70, &arOutputData, 1);
	Send(m_SendBuf, nCmdLenght);
}

int SpeedFixed(int inValue)
{
	if(inValue > 6000)
		return 6000;
	if(inValue < 0)
		return 0;
	return inValue;
}
void CWPB_Mani_driver::SetManiSpd(int inJoint1, int inJoint2, int inJoint3, int inJoint4, int inGripper)
{
	arManiSpeed[0] = SpeedFixed(inJoint1);
	arManiSpeed[1] = SpeedFixed(inJoint2);
	arManiSpeed[2] = SpeedFixed(inJoint3);
	arManiSpeed[3] = SpeedFixed(inJoint4);
	arManiSpeed[4] = SpeedFixed(inGripper);
}

void CWPB_Mani_driver::SendManiPos(int inJoint1, int inJoint2, int inJoint3, int inJoint4, int inGripper)
{
	arManiPos[0] = inJoint1;
	arManiPos[1] = inJoint2;
	arManiPos[2] = inJoint3;
	arManiPos[3] = inJoint4;
	arManiPos[4] = inGripper;
	static unsigned char arManiCmdData[40];
	for (int i = 0; i < 5; i++)
	{
		m_Split4Bytes(arManiCmdData + i * 8, arManiSpeed[i]);
		m_Split4Bytes(arManiCmdData + i * 8 + 4, arManiPos[i]);
	}
	int nCmdLenght = GenCmd(0, 0x41, 0x08, 0x63, arManiCmdData, 40);
	Send(m_SendBuf, nCmdLenght);
}

static float fKMecanumLinearMotorK = -1900;
static float fKMecanumLinearMotorKY = -2200;
static float fKMecanumAngularMotorK = -740;
void CWPB_Mani_driver::Velocity(float inX, float inY, float inAngular)
{
	int nMotorToSend[4];
	//upward backward
	int nTmpMotorVal = inX * fKMecanumLinearMotorK;
	nMotorToSend[0] = -nTmpMotorVal;
	nMotorToSend[1] = nTmpMotorVal;
	nMotorToSend[2] = nTmpMotorVal;
	nMotorToSend[3] = -nTmpMotorVal;

	//shif left right
	nTmpMotorVal = inY * fKMecanumLinearMotorKY;
	nMotorToSend[0] += nTmpMotorVal;
	nMotorToSend[1] += nTmpMotorVal;
	nMotorToSend[2] += -nTmpMotorVal;
	nMotorToSend[3] += -nTmpMotorVal;

	//Turning 
	nTmpMotorVal = inAngular * fKMecanumAngularMotorK;
	nMotorToSend[0] += nTmpMotorVal;
	nMotorToSend[1] += nTmpMotorVal;
	nMotorToSend[2] += nTmpMotorVal;
	nMotorToSend[3] += nTmpMotorVal;

	//printf("[CWPB_driver::Mecanum]-> [0]%d [1]%d [2]%d [3]%d \n", nMotorToSend[0], nMotorToSend[1], nMotorToSend[2], nMotorToSend[3]);

	SendMotors(nMotorToSend[0],nMotorToSend[1],nMotorToSend[2],nMotorToSend[3]);
}

float CWPB_Mani_driver::GetYaw()
{
	float diffYaw = fCurYaw - fFirstYaw;
	return diffYaw;
}


void CWPB_Mani_driver::MotorCmd(int inMethod, int inID1, int inValue1, int inID2, int inValue2)
{
	static unsigned char arMotorSpeedData[12];
	m_Split2Bytes(arMotorSpeedData,inID1);
	m_Split4Bytes(arMotorSpeedData + 2, inValue1);

	m_Split2Bytes(arMotorSpeedData + 6, inID2);
	m_Split4Bytes(arMotorSpeedData + 8, inValue2);
	int nCmdLenght = GenCmd(0, 0x40, 0x08, inMethod, arMotorSpeedData, 12);
	Send(m_SendBuf, nCmdLenght);
}


void CWPB_Mani_driver::MotorCmd2(int inMethod, int inID1, int inValue1_1, int inValue1_2, int inID2, int inValue2_1, int inValue2_2)
{
	static unsigned char arMotorCmdData[20];
	m_Split2Bytes(arMotorCmdData, inID1);
	m_Split4Bytes(arMotorCmdData + 2, inValue1_1);
	m_Split4Bytes(arMotorCmdData + 6, inValue1_2);

	m_Split2Bytes(arMotorCmdData + 10, inID2);
	m_Split4Bytes(arMotorCmdData + 12, inValue2_1);
	m_Split4Bytes(arMotorCmdData + 16, inValue2_2);
	int nCmdLenght = GenCmd(0, 0x40, 0x08, inMethod, arMotorCmdData, 20);
	Send(m_SendBuf, nCmdLenght);
}

bool CWPB_Mani_driver::ManiArrived()
{
	bool bArrived = true;
	// if(abs (arMotorPos[4] - nLastCmdLiftPos) > 100 )
	// {
	// 	bArrived = false;
	// }
	// if(abs (arMotorPos[5] - nLastCmdGripperPos) > 100 )
	// {
	// 	bArrived = false;
	// }
	return bArrived;
}
