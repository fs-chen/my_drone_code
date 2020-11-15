#include "LineInspection.h"


LineInspectObject LineInspect;
//Ѳ�߽ṹ��
uint8_t FindTowerBFlag=0;
//������B�ı�־Bit
uint8_t FindBarCodeFlag=0;
//��������ı�־Bit
uint8_t FindQRCodeFlag=0;
//���ֶ�ά��ı�־Bit
uint8_t TurnCornerFinishFlag=0;
//ת����ɵı�־Bit
uint8_t FindTowerAFlag=0;
//����ʱ����A�ı�־Bit
void LineInspectInit(void)
{
	LineInspect.Stage = AdjustPosA;
	LineInspect.Target.Target_X_V = 0.f;
	LineInspect.Target.Target_Y_V = 0.f;
	LineInspect.Target.Target_H = 1.f;
	LineInspect.Target.Target_Yaw_Acc = 0.f;
}
extern uint8_t NoRegTowerFlagH;
//ˮƽ����ͷδʶ����
extern uint8_t NoRegLineFlagH;
//ˮƽ����ͷδʶ���߱�־λ 1Ϊδʶ�� 0���෴
extern int16_t TowerPosX;
//����X����
extern uint16_t TowerDeltaX;
//����X���
extern int16_t LinePosY;
//�ߵ�Y����
extern uint16_t LineDeltaY;
//�ߵ�Y���
int16_t TowerTarX;
//��������X����
uint16_t TowerTarDeltaX;
//�����������
int16_t LineTarY;
//�ߵ�����Y����
uint16_t LineTarDeltaY;
//�ߵ������߶�

uint8_t LineDeltaYDeathZoon=4;
//���ߵĸ߶ȵ��������
uint8_t TowerXDeathZoon=4;
//������λ���������
uint8_t LinePosYDeathZoon=4;
//���ߵ�λ�ø߶�Y������
uint8_t TowerDeltaXDeathZoon=4;
//�����Ŀ�ȵ��������
float Inspect1Speed=0.01f;
//Ѳ��1�ٶ�
float AdjustXSpeed=0.005f;
//����X��������ٶ�
float AdjustYSpeed=0.005f;
//����Y��������ٶ�
uint8_t AdjustStep=10;
//��������
float AdjustHeightWeight=0.005f;
//�߶ȵ�������

extern DroneRTInfo RT_Info;     
//������ʵʱ����
extern DroneTargetInfo Target_Info;     
//������Ŀ���ȫ�ֱ���
extern DroneFlightControl FlightControl;   
//������״̬����


void TakeOffPro(void)
{
//	if()//����ź�
	{
		float HeightBias=RT_Info.US100_Alt - Target_Info.Height;
		static uint8_t TakeOffCnt=0;
		if(ABS(HeightBias)<0.05f)
		{
			TakeOffCnt++;
			if(TakeOffCnt>=200)
			{
				TakeOffCnt=0;
				LineInspect.Stage = AdjustPosA;  //�����ɺ�,����λ�õ���A
			}
		}
		else
		{
			TakeOffCnt=0;		
		}
	}
}


//��������A
void AdjustAPosPro(void)
{	
	TowerTarX = -50;
//	TowerTarDeltaX = 20;
	//����������
	//�����Ŀ���
//	int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
//	int16_t TowerErroPosX = TowerTarX - TowerPosX;
	int16_t LineErroPosY = LineTarY - LinePosY;
//	int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
	//���������
	//�˴���X������ͷX
//	static uint8_t AdjustCnt=0;
	if(NoRegTowerFlagH!=1)
	{
		//��������
//		if((TowerErroPosX>-TowerXDeathZoon) && (TowerErroPosX<TowerXDeathZoon))
//		{
//			TowerErroPosX=0;			
//		}//����
//		if(TowerErroPosX>0)//�����ڹ���Y���ϡ�
//		{
//			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
//			{
//				LineInspect.Target.Target_Y_V = AdjustYSpeed;
//			}
//		}
//		else if(TowerErroPosX<0)
//		{
//			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
//			{
//				LineInspect.Target.Target_Y_V = -AdjustYSpeed;
//			}			
//		}
//		else
//		{
//			if(LineInspect.Target.Target_Y_V > 0)
//			{
//				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_Y_V<=0)
//				{
//					LineInspect.Target.Target_Y_V = 0;
//				}					
//			}
//			else if(LineInspect.Target.Target_Y_V < 0)
//			{
//				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_Y_V>=0)
//				{
//					LineInspect.Target.Target_Y_V = 0;
//				}
//			}
//			else
//			{
//				LineInspect.Target.Target_Y_V = 0.0f;
//			}

//		}
		

//		TowerTarDeltaX = 10;
//		//���ߵ�����

//		
//		//���������
//		//�˴���Y������ͷY
//		if((TowerErroDeltaX>-TowerDeltaXDeathZoon) && (TowerErroDeltaX<TowerDeltaXDeathZoon))
//		{
//			TowerErroDeltaX=0;			
//		}//����

//		
//		if(TowerErroDeltaX>0) //�����ڹ���X���ϡ�
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
//			{
//				LineInspect.Target.Target_X_V=-AdjustXSpeed;
//			}
//		}
//		else if(TowerErroDeltaX<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
//			{
//				LineInspect.Target.Target_X_V=AdjustXSpeed;
//			}
//		}
//		else
//		{
//			if(LineInspect.Target.Target_X_V>0)
//			{
//				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_X_V<=0)
//				{
//					LineInspect.Target.Target_X_V=0;
//				}
//			}
//			else if(LineInspect.Target.Target_X_V<0)
//			{
//				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_X_V>=0)
//				{
//					LineInspect.Target.Target_X_V=0;
//				}
//			}
//			else
//			{
//				LineInspect.Target.Target_X_V = 0.0f;
//			}
//		}
//		if((ABS(TowerErroPosX)<(2*TowerXDeathZoon)) && (ABS(LineErroPosY)<(2*LinePosYDeathZoon)) && (ABS(TowerErroDeltaX)<(2*TowerDeltaXDeathZoon)))
//		{
//			AdjustCnt++;
//			if(AdjustCnt>=200)
//			{
//				AdjustCnt=0;		
//				LineInspect.Stage = Inspect1;
//			}//�����ȶ��� ��ʼѲ��
//		}
//		else
//		{
//			AdjustCnt=0;
//		}				
		LineInspect.Target.Target_X_V=0;
	}
	else
	{
		//δ���յ���ȷ�����ݰ�
//		if(LineInspect.Target.Target_Y_V > 0)
//		{
//			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V<=0)
//			{
//				LineInspect.Target.Target_Y_V = 0;
//			}					
//		}
//		else if(LineInspect.Target.Target_Y_V < 0)
//		{
//			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V>=0)
//			{
//				LineInspect.Target.Target_Y_V = 0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_Y_V = 0.0f;
//		}
		
		
//		if(LineInspect.Target.Target_X_V>0)
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else if(LineInspect.Target.Target_X_V<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_X_V = 0.0f;
//		}		
	}
	
	if(NoRegLineFlagH!=1)
	{
		//���ߵĿ���
		//ʶ����ȷ������
		LineTarY = 0;
//		ALLLED_ON;
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;			
		}//����		
		if(LineErroPosY>0)//�����ڸ߶���
		{
			//˵�����˻�ƫ�� ��Ҫ��߸߶�
//			LineInspect.Target.Target_H+=AdjustHeightWeight;
//			Target_Info.Height+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//˵�����˻�ƫ�� ��Ҫ���͸߶�
//			LineInspect.Target.Target_H-=AdjustHeightWeight;
//			Target_Info.Height-=AdjustHeightWeight;
		}
		else
		{
//			LineInspect.Target.Target_H+=0.f;
			Target_Info.Height+=0;
		}				
	}
	else
	{
		ALLLED_OFF;
//		//δʶ����ȷ������
//		if(LineInspect.Target.Target_X_V>0)
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else if(LineInspect.Target.Target_X_V<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_X_V = 0.0f;
//		}
//		
		LineInspect.Target.Target_H+=0.f;
	}

}


void Inspect1Pro(void)
{
	if(NoRegLineFlagH!=1)
	{	
		LineTarY=0;
		LineTarDeltaY = 10;
		//���ߵ�����
		int16_t LineErroPosY = LineTarY - LinePosY;
		int16_t LineErroDeltaY = LineTarDeltaY - LineDeltaY;
		//�����߿����
		
		if((LineErroDeltaY>-LineDeltaYDeathZoon) && (LineErroDeltaY<LineDeltaYDeathZoon))
		{
			LineErroDeltaY=0;
			
		}//����	
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;
			
		}//����
		
		if(LineErroDeltaY>0)//�����ڹ���X����
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=-AdjustXSpeed;
			}
		}
		else if(LineErroDeltaY<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}			
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}						
		}
		
		if(LineErroPosY>0)//�����ڸ߶���
		{
			//˵�����˻�ƫ�� ��Ҫ��߸߶�
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//˵�����˻�ƫ�� ��Ҫ���͸߶�
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}	
		
		if(FindTowerBFlag || FindBarCodeFlag)
		{
			//��������B ���� ����
			if(FindBarCodeFlag)//����������ͬʱ����
			{
				//����������
				static uint16_t BeepCnt=0;
				BEEP_ON; //�������� ������������ʾ 
				LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{				
					LineInspect.Target.Target_Y_V=0;						
					BeepCnt++;
					if(BeepCnt>=500)
					{
						BEEP_OFF; //����һ��ر�
						BeepCnt=0;
						FindBarCodeFlag=0;//����־λ��0;
					}
				}
			}
			else if(FindTowerBFlag)
			{	
				//��������B
				LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;			
				}
				static uint8_t FlowYAdjustCnt=0;
				if(ABS(RT_Info.FlowY)<0.03f)
				{
				
					//�������ȶ���һ��ʱ���
					FlowYAdjustCnt++;
					if(FlowYAdjustCnt>=100)
					{	
						FindTowerBFlag=0;
						FlowYAdjustCnt=0;
						LineInspect.Stage = AdjustPosB;
					}
				}
				else
				{
					FlowYAdjustCnt=0;
				
				}
			}
		}
		else
		{
			LineInspect.Target.Target_Y_V += (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=Inspect1Speed)
			{
				LineInspect.Target.Target_Y_V=Inspect1Speed;
			}
		}	
	}
	else
	{
		LineInspect.Target.Target_H+=0;
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V=0;
			}			
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}	
		
		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}			
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}			
	}
}


//��δ�Ѳ�߹��̴�������һ����
void AdjustBPosPro(void)
{
	if(NoRegTowerFlagH!=1)
	{
		//������ȷ
		TowerTarX = 0;
		TowerTarDeltaX = 30;
		//����������
		int16_t TowerErroX = TowerTarX - TowerPosX;
		int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
		//������������ֵ��
		if(ABS(TowerErroX)<TowerXDeathZoon)
		{
			TowerErroX=0;
		}//����
		if(ABS(TowerErroDeltaX)<TowerDeltaXDeathZoon)
		{
			TowerErroDeltaX=0;
		}	//����
		
		
		if(TowerErroX>0) //������Y����
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V=AdjustYSpeed;
			}
		}
		else if(TowerErroX<0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V=-AdjustYSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}
			}
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}		
		}

	
		if(TowerErroDeltaX>0)//������X����
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = -AdjustXSpeed;		
			}
		}
		else if(TowerErroDeltaX<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = AdjustXSpeed;		
			}		
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V = 0;		
				}			
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V = 0;		
				}	
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}
		}
		//������Bλ�õĿ���
		static uint8_t AdjustBCnt=0;
		if((ABS(RT_Info.FlowX_V)<0.03f)&&(ABS(RT_Info.FlowY_V)<0.03f)&&(ABS(TowerErroDeltaX)<(2*TowerDeltaXDeathZoon))&&(ABS(TowerErroX)<(2*TowerXDeathZoon)))
		{
			//λ���ȶ�֮�� �ȴ�Openmv�����ά��
			
			AdjustBCnt++;
			//�����ȶ������

		}
		else
		{
			AdjustBCnt=0;
		}
		if(AdjustBCnt>=100)
		{		
			if(FindQRCodeFlag)
			{
				AdjustBCnt=0;
				FindQRCodeFlag=0;
				LineInspect.Stage=TurnCorner;
				//��ʼת��
			}
		}		
	}
	else
	{
		//δʶ����ȷ��λ��
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}	
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V = 0;		
			}			
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V = 0;		
			}	
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}		
	}	
	
	if(NoRegLineFlagH!=1)
	{
		//ʶ����ȷ����
		LineTarY=0;
		int16_t LineErroPosY = LineTarY - LinePosY;	
		//��A�ĸ߶�����
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;
			
		}//����	
		if(LineErroPosY>0)//�����ڸ߶���
		{
			//˵�����˻�ƫ�� ��Ҫ��߸߶�
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//˵�����˻�ƫ�� ��Ҫ���͸߶�
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}

	}
	else
	{
		LineInspect.Target.Target_H+=0.f;	
	}
}



void TurnCornerPro(void)
{
	if(TurnCornerFinishFlag!=1)
	{
		if(NoRegTowerFlagH!=1)
		{
			//ʶ����ȷ����
			//ת�����
			TowerTarX = 0;
			TowerTarDeltaX = 30;
			//����������
			int16_t TowerErroX = TowerTarX - TowerPosX;
			int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
			//������������ֵ��
			if(ABS(TowerErroX)<TowerXDeathZoon)
			{
				TowerErroX=0;
			}//����
			if(ABS(TowerErroDeltaX)<TowerDeltaXDeathZoon)
			{
				TowerErroDeltaX=0;
			}//����
			
			if(TowerErroX>0)//������Y����
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
				{
					LineInspect.Target.Target_Y_V=AdjustYSpeed;
				}
			}
			else if(TowerErroX<0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
				{
					LineInspect.Target.Target_Y_V=-AdjustYSpeed;
				}
			}
			else
			{
				if(LineInspect.Target.Target_Y_V>0)
				{
					LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V<0)
					{
						LineInspect.Target.Target_Y_V = 0.0f;
					}
				}
				else if(LineInspect.Target.Target_Y_V<0)
				{
					LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V>0)
					{
						LineInspect.Target.Target_Y_V = 0.0f;
					}
				}	
				else
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
				
			}
			
			if(TowerErroDeltaX>0)//������X����
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V = -AdjustXSpeed;			
				}
			}
			else if(TowerErroDeltaX<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V = AdjustXSpeed;	
				}
			}
			else
			{
				if(LineInspect.Target.Target_X_V>0)
				{
					LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);;
					if(LineInspect.Target.Target_X_V<=0)
					{
						LineInspect.Target.Target_X_V=0;
					}
				}
				else if(LineInspect.Target.Target_X_V<0)
				{
					LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);;
					if(LineInspect.Target.Target_X_V>=0)
					{
						LineInspect.Target.Target_X_V=0;
					}		
				}
				else
				{
					LineInspect.Target.Target_X_V = 0.0f;
				}
			}
		}
		else
		{
			//ʶ�𲻵���ȷ�ź�
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);;
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);;
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}		
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}
			
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<0)
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>0)
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
			}	
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}			
		}
	}
	else 
	{
		//ת����� ����λ��
		if(NoRegLineFlagH!=1)
		{
			//ʶ����ȷ����
			LineTarY=0;
			LineTarDeltaY=20;
			int16_t LineErroPosY= LineTarY-LinePosY;
			int16_t LineErroDeltaY = LineTarDeltaY-LineDeltaY;
			if(ABS(LineErroPosY)<LinePosYDeathZoon)
			{
				LineErroPosY=0;	
			}//����
			if(ABS(LineErroDeltaY)<LineDeltaYDeathZoon)
			{
				LineErroDeltaY=0;
			}//����
			
			if(LineErroPosY>0)//�����ڸ߶���
			{
				//˵�����˻�ƫ�� ��Ҫ��߸߶�
				LineInspect.Target.Target_H+=AdjustHeightWeight;
			}
			else if(LineErroPosY<0)
			{
				//˵�����˻�ƫ�� ��Ҫ���͸߶�
				LineInspect.Target.Target_H-=AdjustHeightWeight;
			}
			else
			{
				LineInspect.Target.Target_H+=0.f;
			}
			
			if(LineErroDeltaY>0)	//�����ڹ���X��
			{	
				//˵��Զ������,��Ҫ���������������
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V=-AdjustXSpeed;
				}
			}
			else if(LineErroDeltaY<0)
			{
				//˵����̫����,�������������
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V=AdjustXSpeed;
				}
			}
			else
			{
				if(LineInspect.Target.Target_X_V>0)
				{
					LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_X_V<=0)
					{
						LineInspect.Target.Target_X_V=0;
					}				
				}
				else if(LineInspect.Target.Target_X_V<0)
				{
					LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_X_V>=0)
					{
						LineInspect.Target.Target_X_V=0;
					}
				}
				else
				{
					LineInspect.Target.Target_X_V=0;
				}
			
			}
			static uint8_t StableCnt=0;
			if((LineErroPosY<(2*LinePosYDeathZoon)) && (LineErroDeltaY<(2*LineDeltaYDeathZoon)))
			{	
				//�ȶ���ʱ
				
				StableCnt++;
				if(StableCnt>=40)
				{
					StableCnt=0;
					LineInspect.Stage = Inspect2;
				}				
			}
			else
			{
				StableCnt=0;
			}
		}
		else
		{
			//ʶ�𲻵���ȷ����
			LineInspect.Target.Target_H+=0.f;
			
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}				
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else
			{
				LineInspect.Target.Target_X_V=0;
			}			
		}
	}
}



void Inspect2Pro(void)
{
	if(NoRegLineFlagH!=1)
	{
		LineTarY=0;
		LineTarDeltaY = 10;
		//���ߵ�����
		int16_t LineErroDeltaY = LineTarDeltaY - LineDeltaY;
		//�����߿����
		int16_t LineErroPosY = LineTarY - LinePosY;
		if(ABS(LineErroDeltaY)<LineDeltaYDeathZoon)
		{
			LineErroDeltaY=0;
		}//����
		if(ABS(LineErroPosY)<LinePosYDeathZoon)
		{
			LineErroPosY=0;	
		}//����
		
		if(LineErroDeltaY>0)//�����ڹ���X����
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=-AdjustXSpeed;
			}
		}
		else if(LineErroDeltaY<0)
		{	
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else 
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}		
		}	
		
		if(LineErroPosY>0)//�����ڸ߶���
		{
			//˵�����˻�ƫ�� ��Ҫ��߸߶�
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//˵�����˻�ƫ�� ��Ҫ���͸߶�
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}	
		
		if(FindTowerAFlag)
		{
			LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;			
			}
			static uint8_t FlowYAdjustCnt=0;
			if(ABS(RT_Info.FlowY)<0.03f)
			{
				
				//�������ȶ���һ��ʱ���
				FlowYAdjustCnt++;
				if(FlowYAdjustCnt>=100)
				{	
					FindTowerAFlag=0;
					FlowYAdjustCnt=0;
					LineInspect.Stage = Land;
				}
			}
			else
			{
				FlowYAdjustCnt=0;
			}
			
		}
		else
		{
			LineInspect.Target.Target_Y_V += (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=Inspect1Speed)
			{
				LineInspect.Target.Target_Y_V=Inspect1Speed;
			}
		
		}
	}
	else
	{
		//ʶ�𲻵���ȷ����
		LineInspect.Target.Target_H+=0.f;
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else 
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}			

		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else 
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}			
	}
}


//��δӷ���Ѳ�ߴ������뵽��һ����
void LandPro(void)
{
	if(NoRegTowerFlagH!=1)
	{
		//ʶ����ȷ����
		TowerTarX = 0;
		TowerTarDeltaX = 20;
		//����������

		//�����Ŀ���
		int16_t TowerErroPosX = TowerTarX - TowerPosX;
		int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
		//���������
		//�˴���X������ͷX
		if(TowerErroPosX>0)//�����ڹ���Y���ϡ�
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V >= AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V = AdjustYSpeed;
			}
		}
		else if(TowerErroPosX<0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V <= -AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V = -AdjustYSpeed;
			}			
		}
		else
		{
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V <= 0)
				{
					LineInspect.Target.Target_Y_V = 0;
				}					
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V >=0)
				{
					LineInspect.Target.Target_Y_V = 0;
				}			
			}
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}
		}
		
		if(TowerErroDeltaX>0) //�����ڹ���X����.
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V <= -AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = -AdjustXSpeed;
			}
		}
		else if(TowerErroDeltaX<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V >= AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V <= 0)
				{
					LineInspect.Target.Target_X_V = 0;
				}					
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V >=0)
				{
					LineInspect.Target.Target_X_V = 0;
				}			
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}		
		}
		static uint8_t StableCnt=0;
		if((ABS(RT_Info.FlowX_V)<=0.3f)&&(ABS(RT_Info.FlowY_V)<=0.3f))
		{
			
			StableCnt++;
			if(StableCnt>=200)
			{
				StableCnt=0;
				OS_ERR err;
				FlightControl.landFlag=1;		
				OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err);			
				while(1)
				{
					BEEP_ON;
					delay_ms(50);
					BEEP_OFF;
					delay_ms(20);
					BEEP_ON;
					LED1_ON;
					LED2_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(100);
					BEEP_OFF;
					LED2_ON;
					LED1_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(130);
					BEEP_ON;
					LED3_ON;
					LED1_OFF;
					LED2_OFF;
					LED4_OFF;
					delay_ms(240);
					BEEP_OFF;
					LED4_ON;
					LED2_OFF;
					LED3_OFF;
					LED1_OFF;
					delay_ms(175);
				}					
			}
		
		}
		else
		{
			StableCnt=0;
		}
	}
	else
	{
		//ʶ�𲻵���ȷ����
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V <= 0)
			{
				LineInspect.Target.Target_X_V = 0;
			}					
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V >=0)
			{
				LineInspect.Target.Target_X_V = 0;
			}			
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}
		
		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V <= 0)
			{
				LineInspect.Target.Target_Y_V = 0;
			}					
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V >=0)
			{
				LineInspect.Target.Target_Y_V = 0;
			}			
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}		
		static uint8_t StableCnt2=0;
		if((ABS(RT_Info.FlowX_V)<=0.3f)&&(ABS(RT_Info.FlowY_V)<=0.3f))
		{
			
			StableCnt2++;
			if(StableCnt2>=200)
			{
				StableCnt2=0;
				OS_ERR err;
				FlightControl.landFlag=1;		
				OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err);			
				while(1)
				{
					BEEP_ON;
					delay_ms(50);
					BEEP_OFF;
					delay_ms(20);
					BEEP_ON;
					LED1_ON;
					LED2_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(100);
					BEEP_OFF;
					LED2_ON;
					LED1_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(130);
					BEEP_ON;
					LED3_ON;
					LED1_OFF;
					LED2_OFF;
					LED4_OFF;
					delay_ms(240);
					BEEP_OFF;
					LED4_ON;
					LED2_OFF;
					LED3_OFF;
					LED1_OFF;
					delay_ms(175);
				}					
			}	
		}
		else
		{
			StableCnt2=0;
		}
	}
}


