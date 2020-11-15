/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Data_PC.c
 * 描述   :上传PC端数据格式
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Data_PC.h"

void sendParaInfo(void)
{
	u8 paraToPC[255];
	u8 floatToHex[4];	
	u8 intToHex[4];
	int i=0;

	paraToPC[0]=0X55;
	paraToPC[1]=0XAA;
	paraToPC[2]=0X02;

	/* Pitch PidPara */
	FloatToByte(Para_Info.Pitch.Kp,floatToHex);
	arrycat(paraToPC,3,floatToHex,4);
	FloatToByte(Para_Info.Pitch.Ki,floatToHex);
	arrycat(paraToPC,7,floatToHex,4);
	FloatToByte(Para_Info.Pitch.Kd,floatToHex);
	arrycat(paraToPC,11,floatToHex,4);

	/* Roll PidPara */
	FloatToByte(Para_Info.Roll.Kp,floatToHex);
	arrycat(paraToPC,15,floatToHex,4);
	FloatToByte(Para_Info.Roll.Ki,floatToHex);
	arrycat(paraToPC,19,floatToHex,4);
	FloatToByte(Para_Info.Roll.Kd,floatToHex);
	arrycat(paraToPC,23,floatToHex,4);

	/* Yaw PidPara */
	FloatToByte(Para_Info.Yaw.Kp,floatToHex);
	arrycat(paraToPC,27,floatToHex,4);
	FloatToByte(Para_Info.Yaw.Ki,floatToHex);
	arrycat(paraToPC,31,floatToHex,4);
	FloatToByte(Para_Info.Yaw.Kd,floatToHex);
	arrycat(paraToPC,35,floatToHex,4);
	
	
	/* Height PidPara */
	FloatToByte(Para_Info.Height.Kp,floatToHex);
	arrycat(paraToPC,39,floatToHex,4);
	FloatToByte(Para_Info.Height.Ki,floatToHex);
	arrycat(paraToPC,43,floatToHex,4);
	FloatToByte(Para_Info.Height.Kd,floatToHex);
	arrycat(paraToPC,47,floatToHex,4);
			
	
	/* ratePitch PidPara */
	FloatToByte(Para_Info.ratePitch.Kp,floatToHex);
	arrycat(paraToPC,51,floatToHex,4);
	FloatToByte(Para_Info.ratePitch.Ki,floatToHex);
	arrycat(paraToPC,55,floatToHex,4);
	FloatToByte(Para_Info.ratePitch.Kd,floatToHex);
	arrycat(paraToPC,59,floatToHex,4);

	/* rateRoll PidPara */
	FloatToByte(Para_Info.rateRoll.Kp,floatToHex);
	arrycat(paraToPC,63,floatToHex,4);
	FloatToByte(Para_Info.rateRoll.Ki,floatToHex);
	arrycat(paraToPC,67,floatToHex,4);
	FloatToByte(Para_Info.rateRoll.Kd,floatToHex);
	arrycat(paraToPC,71,floatToHex,4);

	/* rateYaw PidPara */
	FloatToByte(Para_Info.rateYaw.Kp,floatToHex);
	arrycat(paraToPC,75,floatToHex,4);
	FloatToByte(Para_Info.rateYaw.Ki,floatToHex);
	arrycat(paraToPC,79,floatToHex,4);
	FloatToByte(Para_Info.rateYaw.Kd,floatToHex);
	arrycat(paraToPC,83,floatToHex,4);

	
	/* accHeight PidPara */
 	FloatToByte(Para_Info.accHeight.Kp,floatToHex);
	arrycat(paraToPC,87,floatToHex,4);
	FloatToByte(Para_Info.accHeight.Ki,floatToHex);
	arrycat(paraToPC,91,floatToHex,4);
	FloatToByte(Para_Info.accHeight.Kd,floatToHex);
	arrycat(paraToPC,95,floatToHex,4);
	

	for(i=99;i<157;i++)
	{
		paraToPC[i]=0;
	}	
	
	IntToByte(0.0,intToHex);
	arrycat(paraToPC,157,intToHex,2);
	
	for(i=0;i<159;i++)
	{
		paraToPC[159]+=paraToPC[i];
	}
	
	if(USE_BLUETOOTH)
		Uart3_tx(paraToPC,160);
	else
		Uart1_tx(paraToPC,160);
		
}
extern LineInspectObject LineInspect;
extern uint8_t ButtonLineAngleFb;
/*上传实时信息*/
//extern int16_t Xpos,Ypos;
void sendRTInfo(void)
{
  float temp;
	u8 floatToHex[4];		
	u8 dataToPC[64];	
	u8 i=0;

	dataToPC[0]=0X55;
	dataToPC[1]=0XAA;
	dataToPC[2]=0X01;
		
	temp = RT_Info.Pitch - Errangle_Info.fixedErroPitch;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,3,floatToHex,4);

	temp = RT_Info.Roll - Errangle_Info.fixedErroRoll;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,7,floatToHex,4);	

	
	temp = RT_Info.Yaw;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,11,floatToHex,4);

	temp = LineInspect.Target.Target_X_V;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,15,floatToHex,4); //高度数据 滤波过的/未处理的数据
//	printf("feedback heigth:%f\r\n",RT_Info.US100_Alt);
//	temp = (RT_Info.FlowY_V/10);
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,15,floatToHex,4);//pitch = 目标高度
//	temp = ((0.001f*Control_Info.rV));
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,15,floatToHex,4);//pitch = 目标高度

//	temp = RT_Info.FlowY_V;
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,15,floatToHex,4);//pitch = 目标高度

	temp = (RT_Info.batteryVoltage);
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,19,floatToHex,4);//pitch = 目标高度


	
//	temp = RT_Info.FlowY_V * 100;
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,23,floatToHex,4); 

//	temp = Target_Info.Height*100;
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,23,floatToHex,4);//pitch = 目标高度

//	temp = RT_Info.FlowX_V*100;
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,23,floatToHex,4);//pitch = 目标高度
	temp = LineInspect.Target.Target_X_V*100;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,23,floatToHex,4);//pitch = 目标高度
//	temp = RT_Info.US100_Alt_V;
//	FloatToByte(temp,floatToHex);
//	arrycat(dataToPC,23,floatToHex,4);	 //roll = 反馈垂直速度	
	
	for(i=27;i<47;i++)
	{
		dataToPC[i]=0;
	}
	for(i=0;i<47;i++)
	{
		dataToPC[47]+=dataToPC[i];
	}
	
	if(USE_BLUETOOTH)
		Uart3_tx(dataToPC,48);
	else
		Uart1_tx(dataToPC,48);
	SendDataToLCD();
}

extern uint8_t usart2_tx_buf[USART2_TX_LEN];

void SendDataToLCD()
{
	int16_t roll,pitch,yaw,height,flowXV,flowYV;
	uint8_t Hbyte,Lbyte;
	usart2_tx_buf[0]=0xDD;
	roll = (int16_t)RT_Info.Roll*100;
	pitch = (int16_t)RT_Info.Pitch*100;
	yaw = (int16_t)RT_Info.Yaw*100;
	height = (int16_t)(RT_Info.US100_Alt*100);
	flowXV = (int16_t)(RT_Info.FlowX_V*100);
	flowYV = (int16_t)(RT_Info.FlowY_V*100);
	
	if(roll>=0)
	{
		Lbyte = (uint8_t)(((uint16_t)roll)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)roll)>>8));
	}
	else
	{
		Lbyte = (uint8_t)(((uint16_t)(-roll))&0x00ff);
		Hbyte = (uint8_t)(((((uint16_t)(-roll))&0x7f)>>8));
		Hbyte |= 0x80;
	}
	usart2_tx_buf[1]=Hbyte;
	usart2_tx_buf[2]=Lbyte;
	
	
	if(pitch>=0)
	{
		Lbyte = (uint8_t)(((uint16_t)pitch)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)pitch)>>8));
	}
	else
	{
		Lbyte = (uint8_t)(((uint16_t)(-pitch))&0x00ff);
		Hbyte = (uint8_t)(((((uint16_t)(-pitch))&0x7f)>>8));
		Hbyte |= 0x80;
	}
	usart2_tx_buf[3]=Hbyte;
	usart2_tx_buf[4]=Lbyte;	
	
	if(yaw>=0)
	{
		Lbyte = (uint8_t)(((uint16_t)yaw)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)yaw)>>8));
	}
	else
	{
		Lbyte = (uint8_t)(((uint16_t)(-yaw))&0x00ff);
		Hbyte = (uint8_t)(((((uint16_t)(-yaw))&0x7f)>>8));
		Hbyte |= 0x80;
	}
	usart2_tx_buf[5]=Hbyte;
	usart2_tx_buf[6]=Lbyte;	
	

	{
		Lbyte = (uint8_t)(((uint16_t)height)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)height)>>8));
	}
	usart2_tx_buf[7]=Hbyte;
	usart2_tx_buf[8]=Lbyte;

	if(flowXV>=0)
	{
		Lbyte = (uint8_t)(((uint16_t)flowXV)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)flowXV)>>8));
	}
	else
	{
		Lbyte = (uint8_t)(((uint16_t)(-flowXV))&0x00ff);
		Hbyte = (uint8_t)(((((uint16_t)(-flowXV))&0x7f)>>8));
		Hbyte |= 0x80;
	}
	usart2_tx_buf[9]=Hbyte;
	usart2_tx_buf[10]=Lbyte;	

	if(flowYV>=0)
	{
		Lbyte = (uint8_t)(((uint16_t)flowYV)&0x00ff);
		Hbyte = (uint8_t)((((uint16_t)flowYV)>>8));
	}
	else
	{
		Lbyte = (uint8_t)(((uint16_t)(-flowYV))&0x00ff);
		Hbyte = (uint8_t)(((((uint16_t)(-flowYV))&0x7f)>>8));
		Hbyte |= 0x80;
	}
	usart2_tx_buf[11]=Hbyte;
	usart2_tx_buf[12]=Lbyte;		
	imu_usart_dma_tx_enable(13);
}





