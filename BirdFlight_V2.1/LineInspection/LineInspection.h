#ifndef __LINEINSPECTION_H
#define __LINEINSPECTION_H


#include "Task.h"


void LineInspectInit(void);


typedef enum
{
	Takeoff=0,									//�����ɱ�׼���ĸ����������һ����Χ�ڲ��ұ���һ��ʱ�䡣
	
	
	AdjustPosA=1,    //У׼��׼��������Ĳ��������룩�Լ�����λ�ã�λ�ã��ķ����ȶ���һ����Χ�ڲ��ұ���һ��ʱ��														//										����������ͷ�Ƕȱ궨��90���Ҳ��ұ���һ��ʱ��
														//�ҵ��ߣ��ߵ�����������ͷ���Ĵ���Χ������߶ȣ������ȶ�
														//�ߵ�����������룩�����ȶ��������ȶ���
	Inspect1=2,									//Ѳ�ߣ��ȴ�openmv����ʶ��������źŽ���������ʾ��

	
	AdjustPosB=3,								//������B��λ�ù�ϵ��������ȴ�openmv�����źš��ȴ�ת���ź�
	
	
	TurnCorner=4,		
	
	
	Inspect2=5,	
	
	
	Land=6
}StageType;


typedef struct
{
	struct
	{		
		float Target_X_V;
		float Target_Y_V;
		float Target_H;
		float Target_Yaw_Acc;
	}Target;
	StageType Stage;

}LineInspectObject;
//Ѱ�߹��̶���


//Ѳ�߹���
void LineInspectInit(void);
void TakeOffPro(void);
void AdjustAPosPro(void);
void Inspect1Pro(void);
void AdjustBPosPro(void);
void TurnCornerPro(void);
void Inspect2Pro(void);
void LandPro(void);

#endif

