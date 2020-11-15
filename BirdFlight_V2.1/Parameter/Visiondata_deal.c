/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech *********************
 * ���� 	:Xiluna Tech
 * �ļ��� :Visiondata_deal.c
 * ����   :�����Ӿ�����������
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
******************************************************************************/
#include "Visiondata_deal.h"


u8 BlackspotsFlag = 1;
u8 OpticalflowFlag = 0;
//volatile float Pix_Xinfo;
//volatile float Pix_Yinfo;
//volatile float OpticalFlow_x;
//volatile float OpticalFlow_y;
//volatile float LastOpticalFlow_x;
//volatile float LastOpticalFlow_y;
//volatile float OpticalFlow_integralx;
//volatile float OpticalFlow_integraly;
//volatile float OpticalFlow_filterx;
//volatile float OpticalFlow_filtery;
//static int PointX_Data[20];
//static int PointY_Data[20];
//static int Flowvx_Data[20];
//static int Flowvy_Data[20];


extern uint8_t HorizonVisionErroFlag;
extern uint8_t ButtonVisionErroFlag;
extern uint16_t ButtonVisionHeartCnt; //������ʱ����Timer3_Ahrs_Inner.c��ʱ��3�ж���
extern uint16_t HorizonVisionHeartCnt; //������ʱ����Timer3_Ahrs_Inner.c��ʱ��3�ж���
extern LineInspectObject LineInspect;
//Ѳ�߽ṹ����Ӿ����ܹ���
extern uint8_t FindTowerAFlag;
//����ʱ����A�ı�־Bit
extern uint8_t FindTowerBFlag;
//������B�ı�־Bit
extern uint8_t FindBarCodeFlag;
//��������ı�־Bit
extern uint8_t FindQRCodeFlag;
//���ֶ�ά��ı�־Bit
int16_t TowerPosX=0;
//����X����
uint16_t TowerDeltaX=0;
//����X���
int16_t LinePosY=0;
//�ߵ�Y����
uint16_t LineDeltaY=0;
//�ߵ�Y���
uint8_t NoRegTowerFlagH=0;
//ˮƽ����ͷδʶ�𵽱�־λ 1Ϊδʶ�� 0���෴
uint8_t NoRegTowerCntH=0;
//ˮƽ����ͷδʶ�����ֵ
uint8_t NoRegLineFlagH=0;
//ˮƽ����ͷδʶ�𵽱�־λ 1Ϊδʶ�� 0���෴
uint8_t NoRegLineCntH=0;
//ˮƽ����ͷδʶ�����ֵ

uint8_t NoRegBarCodeFlagH=0;
//ˮƽ����ͷδʶ�𵽱�־λ 1Ϊδʶ�� 0���෴
uint8_t NoRegBarCodeCntH=0;
//ˮƽ����ͷδʶ�����ֵ

uint8_t NoRegQRCodeFlagH=0;
//ˮƽ����ͷδʶ�𵽱�־λ 1Ϊδʶ�� 0���෴
uint8_t NoRegQRCodeCntH=0;
//ˮƽ����ͷδʶ�����ֵ
int TowerXPosMedianArray[15]={0};
//
int TowerDeltaXMedianArray[15]={0};
//
int LineYPosMedianArray[15]={0};
//
int LineDeltaYMedianArray[15]={0};
//
int BarCodeXMedianArray[15]={0};
//
int BarCodeYMedianArray[15]={0};
//
int QRCodeXMedianArray[15]={0};
//
int QRCodeYMedianArray[15]={0};
//
int16_t BarCodeX,BarCodeY;
int16_t QRCodeX,QRCodeY;
extern uint8_t TurnFlag,landflag;
void VisionDataDealHorizon(_Data_Rx rx)
{
	static uint8_t FindBCnt=0;
	static uint8_t FindACnt=0;
	uint16_t tempx=0,tempy=0;
	static uint8_t frame_cnt=0;
	if(rx.buf[0] == 0xDD)
	{
		RequireFlag=0;
		HorizonVisionHeartCnt=0;
		HorizonVisionErroFlag=0;
		frame_cnt++;
		if(rx.buf[1]==0x01)
		{
			if(rx.buf[2]==0xAA)//��ʶ��ɹ�
			{				
				landflag=1;
				tempx = ((((uint16_t)rx.buf[3])<<8)|rx.buf[4]);
				if(tempx & 0x8000)//����
				{
					tempx &= 0x7fff;
					TowerPosX = -tempx;
				}		
				else//����
				{
					TowerPosX = tempx;
				}				
				TowerDeltaX = ((((uint16_t)rx.buf[5])<<8)|rx.buf[6]);				
				//��ֵ�˲�
				TowerPosX = Median_filter(TowerPosX,15,TowerXPosMedianArray);							
				TowerDeltaX = Median_filter(TowerDeltaX,15,TowerDeltaXMedianArray);					
//				if(LineInspect.Stage==Inspect1)
//				{				
//					if(TowerPosX>=0)
//						FindBCnt++;
//					if(FindBCnt>=5)
//					{
//						FindBCnt=0;
//						FindTowerBFlag=1;
//					}
//					//Ѳ�߼��ģʽ
//				}
//				if(LineInspect.Stage==Inspect2)
//				{
//					
//					if(TowerPosX>=0)
//						FindACnt++;
//					if(FindACnt>=5)
//					{
//						FindACnt=0;
//						FindTowerAFlag=1;
//					}
//					//Ѳ�߷���ģʽ
//				}
			}
			else//��ʶ��ʧ��
			{
				RequireFlag=0;
				NoRegTowerCntH++;		
				if(LineInspect.Stage==Inspect1)
				{
					FindBCnt--;
					//����֡���ж��Ƿ�Ϊʶ��B
				}
				if(LineInspect.Stage==Inspect2)
				{
					FindACnt--;
					//����֡���ж��Ƿ�Ϊʶ��A
				}

			}		
			if(rx.buf[7]==0xAA)//��ʶ��
			{
				tempy = ((((uint16_t)rx.buf[8])<<8)|rx.buf[9]);
				if(tempy & 0x8000)//����
				{
					tempy &= 0x7fff;
					LinePosY = -tempy;
				}		
				else//����
				{
					LinePosY = tempy;
				}
				LineDeltaY = ((((uint16_t)rx.buf[10])<<8)|rx.buf[11]);							
				//��ֵ�˲�
				LinePosY = Median_filter(LinePosY,15,LineYPosMedianArray);							
				LineDeltaY = Median_filter(LineDeltaY,15,LineDeltaYMedianArray);							
			}
			else
			{
				NoRegLineCntH++;			
			}		
		}
		else if(rx.buf[1]==0x02)
		{
			if(rx.buf[2]==0xAA)
			{
				uint16_t tempx,tempy;
				tempx =  ((((uint16_t)rx.buf[2])<<8)|rx.buf[3]);							
				tempy =  ((((uint16_t)rx.buf[4])<<8)|rx.buf[5]);		
				if(tempx & 0x8000)//����
				{
					tempx &= 0x7fff;
					BarCodeX = -tempx;
				}		
				else//����
				{
					BarCodeX = tempx;
				}		
				
				if(tempy & 0x8000)//����
				{
					tempy &= 0x7fff;
					BarCodeY = -tempy;
				}		
				else//����
				{
					BarCodeY = tempy;
				}					
				BarCodeX = Median_filter(BarCodeX,15,BarCodeXMedianArray);		
				BarCodeY = Median_filter(BarCodeY,15,BarCodeYMedianArray);		
				FindBarCodeFlag=1;	
			}
			else
			{
			
			
			}
		}	
		else if(rx.buf[1]==0x03)
		{
			if(rx.buf[2]==0xAA)
			{
				uint16_t tempx,tempy;
				tempx =  ((((uint16_t)rx.buf[2])<<8)|rx.buf[3]);							
				tempy =  ((((uint16_t)rx.buf[4])<<8)|rx.buf[5]);		
				if(tempx & 0x8000)//����
				{
					tempx &= 0x7fff;
					QRCodeX = -tempx;
				}		
				else//����
				{
					QRCodeX = tempx;
				}		
				
				if(tempy & 0x8000)//����
				{
					tempy &= 0x7fff;
					QRCodeY = -tempy;
				}		
				else//����
				{
					QRCodeY = tempy;
				}					
				QRCodeX = Median_filter(QRCodeX,15,QRCodeXMedianArray);		
				QRCodeY = Median_filter(QRCodeY,15,QRCodeYMedianArray);					
				FindQRCodeFlag=1;
				//ʶ������
			}
			else
			{
			}
		}
		else if(rx.buf[1]==0x04)
		{
			FindBarCodeFlag=0;  //����������ɼ���Ѳ��
		}
		else if(rx.buf[1]==0x05)
		{
			TurnFlag=1;
			FindQRCodeFlag=0; 	//��ά��������� ��ʼת��		
		}
		else
		{
			NoRegTowerCntH++;
			NoRegLineCntH++;	
		}
		//֡��ʶ�� �ж���Ϣ���Ŷ�
		if(frame_cnt>=10)
		{
			if(NoRegTowerCntH>=5)//��ʶ����Ŷȵ�
			{						
				for(uint8_t i=0;i<15;i++)
				{
					TowerXPosMedianArray[i]=0;
					TowerDeltaXMedianArray[i]=0;
				}
				NoRegTowerFlagH=1;
			}
			else
			{
				NoRegTowerFlagH=0;				
			}
//			
//			if(NoRegLineCntH>=5)//��ʶ����Ŷȵ�
//			{
//				for(uint8_t j=0;j<15;j++)
//				{
//					LineYPosMedianArray[j]=0;
//					LineDeltaYMedianArray[j]=0;
//				}
//				NoRegLineFlagH=1;					
//			}	
//			else
//			{
//				NoRegLineFlagH=0;				
//			}

			if(NoRegBarCodeCntH>=5)//����ʶ����Ŷȵ�
			{
				for(uint8_t j=0;j<15;j++)
				{
					BarCodeXMedianArray[j]=0;
					BarCodeYMedianArray[j]=0;
				}
				NoRegBarCodeFlagH=1;					
			}	
			else
			{
				NoRegBarCodeFlagH=0;				
			}
			
			if(NoRegQRCodeCntH>=5)//��ά��ʶ����Ŷȵ�
			{
				for(uint8_t j=0;j<15;j++)
				{
					QRCodeXMedianArray[j]=0;
					QRCodeYMedianArray[j]=0;
				}
				NoRegQRCodeFlagH=1;					
			}	
			else
			{
				NoRegQRCodeFlagH=0;				
			}
			
			NoRegBarCodeCntH=0;
			NoRegQRCodeCntH=0;
			NoRegTowerCntH=0;
			NoRegLineCntH=0;
			frame_cnt=0;
		}		
//		printf("TDX:%d\tTX:%d\tLDY:%d\tLY:%d\r\n",TowerDeltaX,TowerPosX,LineDeltaY,LinePosY);
	}
}

int CycleXMedianArray[15]={0};
int CycleYMedianArray[15]={0};
uint8_t ButtonLineAngleFb;
uint8_t NoRegFlagB=0;//�ײ�δʶ���־λ
uint8_t NoRegCntB=0;	 //�ײ�δʶ�𵽵�֡��

int16_t Xpos=0,Ypos=0; 

uint8_t ButtonRequireFlag=0;
void VisionDataDealButton(_Data_Rx rx)
{
	
	static uint8_t FrameCnt=0;
	uint16_t tempx=0,tempy=0;
	if(rx.buf[0]==0xDD)
	{
		ButtonRequireFlag=0;
		ButtonVisionErroFlag=0;
		ButtonVisionHeartCnt=0;		
		FrameCnt++;
		if(rx.buf[1]==0xAA)//ʶ����
		{
			tempx=((((uint16_t)rx.buf[2])<<8)|rx.buf[3]);	
			tempy= ((((uint16_t)rx.buf[4])<<8)|rx.buf[5]);		
			if(tempx&0x8000)
			{
				tempx&=0x7fff;
				Xpos=-tempx;
			}
			else
			{
				Xpos=tempx;
			}
			
			if(tempy&0x8000)
			{
				tempy&=0x7fff;
				Ypos=-tempy;
			}
			else
			{
				Ypos=tempy;
			}
			Xpos = Median_filter(Xpos,15,CycleXMedianArray);		
			Ypos = Median_filter(Ypos,15,CycleYMedianArray);	
//			printf("X:%d\tY:%d\r\n",Xpos,Ypos);

		}
		else//ûʶ��
		{
			NoRegCntB++;
			
		}
		
		//֡���ж�
		if(FrameCnt>=10)
		{
			if(NoRegCntB>=10) //ʶ�𲻿��� ����Ӧ����
			{
				for(uint8_t i=0;i<15;i++)
				{
					CycleXMedianArray[i]=0;//�����ֵ�˲�����
					CycleYMedianArray[i]=0;//�����ֵ�˲�����
				}
				NoRegFlagB=1;//ʶ����̫�� ���ݲ�����
			}	
			else
			{
				NoRegFlagB=0;	//ʶ��������Ҫ�����ݿ���
			}
			
			NoRegCntB=0;
			FrameCnt=0;
		}	
	}
}


/******ԭʼ�汾*******/
//void Vision_datadeal(_Data_Rx rx)
//{
//	int Blob_num = 11;
//	int Flow_num = 11;
//	int tmp;
//	if(rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA)
//	{
//		/*�����Ӿ�ģ�鷢������Ϣ*/
//		if(rx.buf[2]==0x10)
//		{	
//			if(rx.buf[3]==0x01)//������
//			{
//				/*  Target_Roll */
//				tmp = ((int16_t)rx.buf[7]<<8) + rx.buf[8];
//				Pix_Xinfo = ((Median_filter(tmp,Blob_num,PointX_Data))*0.0001f - ImageCenter_x) * US100_Altinfo;
//				/*  Target_Pitch */
//				tmp = ((int16_t)rx.buf[9]<<8) + rx.buf[10];
//				Pix_Yinfo = ((Median_filter(tmp,Blob_num,PointY_Data))*0.0001f - ImageCenter_y) * US100_Altinfo;	
//				BlackspotsFlag = 1;
//				OpticalflowFlag = 0;
//			}
//			else if(rx.buf[3]==0x02)//��������
//			{
//				/* ��ȡ��ǰflow���� */
//				tmp = ((int16_t)rx.buf[7]<<8) + rx.buf[8];
//				if(tmp>=32768)
//				{
//					tmp = 32768 - tmp;
//				}
//				else
//				{
//					tmp = tmp;
//				}
//				OpticalFlow_x = (Median_filter(tmp,Flow_num,Flowvx_Data))*0.0005f;
//				
//				tmp = ((int16_t)rx.buf[9]<<8) + rx.buf[10];
//				if(tmp>=32768)
//				{
//					tmp = 32768 - tmp;
//				}
//				else
//				{
//					tmp = tmp;
//				}
//				OpticalFlow_y = (Median_filter(tmp,Flow_num,Flowvy_Data))*0.0005f;
////				OpticalFlow_y = ((float)tmp)*0.0001f;//0.0001f;
//				
//								
////				/* ����shitomasi������������ٶȵ�ͻ�䣬���м��˲����� */
////				if((OpticalFlow_x - LastOpticalFlow_x) > 0.05f || (OpticalFlow_x - LastOpticalFlow_x) < -0.05f)
////				{
////					OpticalFlow_filterx = 0;
////					OpticalFlow_filtery = 0;
////				}
////				else
////				{
////					OpticalFlow_filterx = OpticalFlow_x - LastOpticalFlow_x;
////					OpticalFlow_filtery = OpticalFlow_y - LastOpticalFlow_y;
////				}
////				/* λ���ۼ� */
////				OpticalFlow_integralx += OpticalFlow_filterx * US100_Altinfo;
////				OpticalFlow_integraly += OpticalFlow_filtery * US100_Altinfo;
////				/* ��ȡ��һ��flow����� */
////				LastOpticalFlow_x = OpticalFlow_x;
////				LastOpticalFlow_y = OpticalFlow_y;
////				
//				BlackspotsFlag = 0;
//				OpticalflowFlag = 1;
//			}
//			else if(rx.buf[3]==0x03)//�����ź�
//			{
//				FlightControl.landFlag=1;
//			}						
//		}		
//	}
//}




