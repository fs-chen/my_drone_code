/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Flash.c
 * ����   :�ڲ�Flash�����ļ�
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Flash.h"

FlashData flashData;
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

//����ǰ����д��flash
void Write_config(void)
{																
	u32 *ptr = &flashData.isGood;
	flashData.pidPara=Para_Info;
	flashData.Errangle=Errangle_Info;
	STMFLASH_Write(0X080E0004,ptr,sizeof(flashData));
}

void load_config(void){
	
	u32 *ptr = &flashData.isGood;
	STMFLASH_Read(0X080E0004,ptr,sizeof(flashData));
	
	if(flashData.isGood==0xA55A5AA5)//�׵�ַ��ȷ
	{		
		Para_Info.Pitch.Kp=flashData.pidPara.Pitch.Kp;
		Para_Info.Pitch.Ki=flashData.pidPara.Pitch.Ki;
		Para_Info.Pitch.Kd=flashData.pidPara.Pitch.Kd;
		
		Para_Info.Roll.Kp=flashData.pidPara.Roll.Kp;
		Para_Info.Roll.Ki=flashData.pidPara.Roll.Ki;
		Para_Info.Roll.Kd=flashData.pidPara.Roll.Kd;
		
		Para_Info.Yaw.Kp=flashData.pidPara.Yaw.Kp;
		Para_Info.Yaw.Ki=flashData.pidPara.Yaw.Ki;
		Para_Info.Yaw.Kd=flashData.pidPara.Yaw.Kd;	
		
//		Para_Info.Yaw.Kp=0.8f;
//		Para_Info.Yaw.Ki=0.004f;
//		Para_Info.Yaw.Kd=0.08f;			

		Para_Info.ratePitch.Kp=flashData.pidPara.ratePitch.Kp;
		Para_Info.ratePitch.Ki=flashData.pidPara.ratePitch.Ki;
		Para_Info.ratePitch.Kd=flashData.pidPara.ratePitch.Kd;
		
		Para_Info.rateRoll.Kp=flashData.pidPara.rateRoll.Kp;
		Para_Info.rateRoll.Ki=flashData.pidPara.rateRoll.Ki;
		Para_Info.rateRoll.Kd=flashData.pidPara.rateRoll.Kd;
		
		Para_Info.rateYaw.Kp=flashData.pidPara.rateYaw.Kp;
		Para_Info.rateYaw.Ki=flashData.pidPara.rateYaw.Ki;
		Para_Info.rateYaw.Kd=flashData.pidPara.rateYaw.Kd;	
		

		Para_Info.Height.Kp=flashData.pidPara.Height.Kp;
		Para_Info.Height.Ki=flashData.pidPara.Height.Ki;
		Para_Info.Height.Kd=flashData.pidPara.Height.Kd;	
		
		Para_Info.accHeight.Kp=flashData.pidPara.accHeight.Kp;
		Para_Info.accHeight.Ki=flashData.pidPara.accHeight.Ki;
		Para_Info.accHeight.Kd=flashData.pidPara.accHeight.Kd;	
		
		
		Errangle_Info.fixedErroPitch = flashData.Errangle.fixedErroPitch;
		Errangle_Info.fixedErroRoll = flashData.Errangle.fixedErroRoll;
		
	}
	else//��һ�ζ�д���׵�ַ����
	{
		flashData.isGood=0xA55A5AA5;
		Para_Info.Pitch.Kp=flashData.pidPara.Pitch.Kp=0;
		Para_Info.Pitch.Ki=flashData.pidPara.Pitch.Ki=0;
		Para_Info.Pitch.Kd=flashData.pidPara.Pitch.Kd=0;
		
		Para_Info.Roll.Kp=flashData.pidPara.Roll.Kp=0;
		Para_Info.Roll.Ki=flashData.pidPara.Roll.Ki=0;
		Para_Info.Roll.Kd=flashData.pidPara.Roll.Kd=0;
		
		Para_Info.Yaw.Kp=flashData.pidPara.Yaw.Kp=0;
		Para_Info.Yaw.Ki=flashData.pidPara.Yaw.Ki=0;
		Para_Info.Yaw.Kd=flashData.pidPara.Yaw.Kd=0;

		Para_Info.Height.Kp=flashData.pidPara.Height.Kp=0;
		Para_Info.Height.Ki=flashData.pidPara.Height.Ki=0;
		Para_Info.Height.Kd=flashData.pidPara.Height.Kd=0;	
		
		
		Para_Info.ratePitch.Kp=flashData.pidPara.ratePitch.Kp=0;
		Para_Info.ratePitch.Ki=flashData.pidPara.ratePitch.Ki=0;
		Para_Info.ratePitch.Kd=flashData.pidPara.ratePitch.Kd=0;
		
		Para_Info.rateRoll.Kp=flashData.pidPara.rateRoll.Kp=0;
		Para_Info.rateRoll.Ki=flashData.pidPara.rateRoll.Ki=0;
		Para_Info.rateRoll.Kd=flashData.pidPara.rateRoll.Kd=0;
		
		Para_Info.rateYaw.Kp=flashData.pidPara.rateYaw.Kp=0;
		Para_Info.rateYaw.Ki=flashData.pidPara.rateYaw.Ki=0;
		Para_Info.rateYaw.Kd=flashData.pidPara.rateYaw.Kd=0;
		
			
		Para_Info.accHeight.Kp=flashData.pidPara.accHeight.Kp=0;
		Para_Info.accHeight.Ki=flashData.pidPara.accHeight.Ki=0;
		Para_Info.accHeight.Kd=flashData.pidPara.accHeight.Kd=0;	
		
		
		Errangle_Info.fixedErroPitch = flashData.Errangle.fixedErroPitch = 0;
		Errangle_Info.fixedErroRoll = flashData.Errangle.fixedErroRoll = 0;
		
		
		/*�˴�ֻ���ڲ��ԣ�WIFI��Ϣ�����ߴ�������*/
	
		Write_config();	
	}
}


