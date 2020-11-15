/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Outer_control.c
 * ����   :�⻷���ƺ���
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Outer_control.h"

PIDOut pidPitch,pidRoll,pidYaw,pidHeight,pidPointX,pidPointY,pidFlowX,pidFlowY;

PIDOut pidOptical;

uint8_t takeoff_suce_flag=0;//////////////////////////////////
static volatile float rollErroHistory=0;
static volatile float pitchErroHistory=0;
static volatile float yawErroHistory=0;
static volatile float tgtHeight=0;
static volatile float tarOptX_V=0,tarOptY_V=0;

void Outer_pidinit()
{
	takeoff_suce_flag=0;////////////////////////////
	Target_Info.Height = 1.0f;//��ʼĿ��߶�
	FlightControl.landFlag = 0;
	pitchErroHistory = 0;
	rollErroHistory = 0;
	yawErroHistory = 0;
	tgtHeight = 0;
	StartFly = 0;
}
extern PIDOut pidFlowx,pidFlowy;

void AttitudeOuter_control()
{
	//��ȷʱ��dt����
	static unsigned int OutertPre=0;
	unsigned int Outert;
	float OuterPID_dt;
	Outert=micros();
  OuterPID_dt = (OutertPre>0)?((Outert-OutertPre)/1000000.0f):1;
  OutertPre=Outert;
	
	/************************ �⻷Pitch�ǶȻ�����************************/	
	float pitchErro=(Target_Info.Pitch-(RT_Info.Pitch-Errangle_Info.fixedErroPitch));
	pidPitch.pOut=Para_Info.Pitch.Kp *  pitchErro;
	pidPitch.dOut=Para_Info.Pitch.Kd * (pitchErro- pitchErroHistory)/ OuterPID_dt;
	pitchErroHistory=pitchErro;
	/*�޷�Pitch�⻷PID*/
	pidPitch.value=pidPitch.pOut+pidPitch.dOut;
	pidPitch.value=Limits_data(pidPitch.value,20000,-20000);
	
	/************************ �⻷Roll�ǶȻ�����************************/	
	float rollErro=(Target_Info.Roll-(RT_Info.Roll-Errangle_Info.fixedErroRoll));
	pidRoll.pOut=Para_Info.Roll.Kp * rollErro;
	pidRoll.dOut=Para_Info.Roll.Kd * (rollErro- rollErroHistory)/ OuterPID_dt;
	rollErroHistory=rollErro;
	/*�޷�Roll�⻷PID*/
	pidRoll.value=pidRoll.pOut+pidRoll.dOut;
	pidRoll.value=Limits_data(pidRoll.value,20000,-20000);
	
	/************************ �⻷Yaw�ǶȻ�����************************/	
	//ԭʼ�汾
	float yawErro=(Target_Info.Yaw-RT_Info.Yaw);
	pidYaw.pOut=Para_Info.Yaw.Kp * yawErro;
	pidYaw.dOut=Para_Info.Yaw.Kd * (yawErro- yawErroHistory)/ OuterPID_dt;
	yawErroHistory=yawErro;
	/*�޷�Yaw�⻷PID*/
	pidYaw.value=pidYaw.pOut+pidYaw.dOut;
	pidYaw.value=Limits_data(pidYaw.value,20000,-20000);

	//

}
extern LineInspectObject LineInspect;
#define Area 45.f
float YspeedWeight=0;
void PostionOuter_control()
{
	/************************ �߶��⻷����************************/
	static float Takeoff_weight = 0.03;
	static float Landing_weight = 0.005;
	if(FlightControl.landFlag==1)
	{
		tgtHeight=tgtHeight - Landing_weight;		
		if(RT_Info.US100_Alt<0.10f)
		{
			FlightControl.OnOff = Drone_Land;
		}
	}
	/*********************�������********************/	
	else
	{
		if(tgtHeight < Target_Info.Height)
		{
			tgtHeight = tgtHeight + Takeoff_weight;
		}
		else
		{
			
			tgtHeight = Target_Info.Height;
		}
	}	
	float Feedforward_height = 35;
	float heightErro = tgtHeight - RT_Info.US100_Alt;
	//ǰ��У׼
	pidHeight.feedforwardOut = Feedforward_height * heightErro;
	
	pidHeight.value = Neurons_PID_Hight(heightErro);

	pidHeight.value = Limits_data(pidHeight.value,3,-3);
	
///**************************	���� ******************************/
////	static float op_Erro=0.f,op_preErro=0.f;
////	op_Erro = tarOptX_V - 
////	pidOptical.feedforwardOut = 

///**********���²����Ƕ���������ݣ�����ʵ�ʵķ��������޸�***************/	
////	if(RT_Info.US100_Alt>0.1f && BlackspotsFlag == 1 && OpticalflowFlag == 0)
////	{	
////		//��������ʶ��ģʽ���ҳ������������ݴ���10cm
////		//�ڵ�ʶ��ģʽ
////		/***************X��PID����***************/
////		//��ͼ�����ĵ����������
////		float Feedforward_Pointx = 1.0f;
////		float PointxErro = RT_Info.PointX;
////		
////		pidPointX.feedforwardOut = Feedforward_Pointx * Pix_Xinfo;
////		//ǰ��У׼
////		pidPointX.value = Neurons_PID_Postionx(PointxErro);
////		//����ԪPID
////		pidPointX.value = Limits_data(pidPointX.value,3,-3);	
////		//PID����޷�
////		
////		/***************Y��PID����***************/	
////		//��ͼ�����ĵ����������	
////		float Feedforward_Pointy = 1.0f;
////		float PointyErro = RT_Info.PointY;
////		
////		pidPointY.feedforwardOut = Feedforward_Pointy * Pix_Yinfo;
////		//ǰ��У׼
////		pidPointY.value = Neurons_PID_Postiony(PointyErro);
////		//����ԪPID
////		pidPointY.value = Limits_data(pidPointY.value,3,-3);	
////		//PID����޷�
////	}
//	{	
//		//target x ������ == roll �ĸ�����
//		//target y ������ == pitch�ĸ�����
//		
//		//����X ������ == ����ͷ����X ������
//		//����Y ������ == ����ͷ����Y ������
//		
//		//����ʶ��ģ��
////		float OpticalFlow_xKp = 3.5f,OpticalFlow_yKp = 3.5f;
////		float Feedforward_flowx = 2.75f,Feedforward_flowy = 2.75f;
////		float OpticalFlow_xKp = 1.f,OpticalFlow_yKp = 1.f;
////		float OpticalFlow_xKd = 0.00f,OpticalFlow_yKd = 0.00f;	
////		static float Last_FlowY=0,Last_FlowX=0;
////		float speed=0.10f;	float speedup_step=350;
////		float speed_h=0.12f; float speedup_step1=400;
//		//����
//		float speed=0.05f;	float speedup_step=100;
//		float speed_h=0.05f; float speedup_step1=100;
//		//ѭ��
//			//��ȷʱ��dt����
////		float OuterPID_dt;
////		unsigned int Outert,OutertPre;
////		Outert=micros();
////		OuterPID_dt = (OutertPre>0)?((Outert-OutertPre)/1000000.0f):1;
////		OutertPre=Outert;
//		//record the last flow data
//		//�⻷λ�ÿ��ƻ�
//		RT_Info.FlowX=0;
//		RT_Info.FlowY=0;
//		//
//		if(RT_Info.US100_Alt>=0.8f)
//		{	
//			{								
//				float FlowxErro = RT_Info.FlowX;
//				
//				//X
//				if(FlowxErro>Area)
//				{
//					if(pidFlowX.value<speed_h)
//					{
//						pidFlowX.value+=(speed_h/(float)speedup_step1);
//						if(pidFlowX.value>=speed_h)
//						{
//							pidFlowX.value=speed_h;						
//						}
//					}
//					else if(pidFlowX.value>speed_h)
//					{
//						pidFlowX.value-=(speed_h/(float)speedup_step1);
//						if(pidFlowX.value<=speed_h)
//						{
//							pidFlowX.value=speed_h;						
//						}					
//					}
//				}//���� [0.05,��]����ʽ
//				else if((FlowxErro<=Area)&&(FlowxErro>0))
//				{
//					if(pidFlowX.value>speed)
//					{
//						pidFlowX.value-=(speed/(float)speedup_step);
//						if(pidFlowX.value<speed)
//						{
//							pidFlowX.value=speed;
//						}
//					}
//					else if(pidFlowX.value<speed)
//					{
//						pidFlowX.value+=(speed/(float)speedup_step);
//						if(pidFlowX.value>speed)
//						{
//							pidFlowX.value=speed;
//						}
//					}	
//				}//���� [0,0.05]����ʽ		
//				else if((FlowxErro>=-Area)&&(FlowxErro<0))
//				{
//					if(pidFlowX.value>=-speed)
//					{
//						pidFlowX.value-=(speed/(float)speedup_step);
//						if(pidFlowX.value<=-speed)
//						{
//							pidFlowX.value=-speed;
//						}
//					}
//					else if(pidFlowX.value<-speed)
//					{
//						pidFlowX.value+=(speed/(float)speedup_step);
//						if(pidFlowX.value>-speed)
//						{
//							pidFlowX.value=-speed;
//						}
//					}	
//				}//���� [-0.05,0]����ʽ		
//				else if((FlowxErro<-Area))
//				{
//					if(pidFlowX.value>=-speed_h)
//					{
//						pidFlowX.value-=(speed_h/(float)speedup_step1);
//						if(pidFlowX.value<=-speed_h)
//						{
//							pidFlowX.value=-speed_h;
//						}
//					}
//					else if(pidFlowX.value<-speed_h)
//					{
//						pidFlowX.value+=(speed_h/(float)speedup_step1);
//						if(pidFlowX.value>-speed_h)
//						{
//							pidFlowX.value=-speed_h;
//						}
//					}	
//				}//���� [-��,-0.05]����ʽ	
//				else
//				{
//					if(pidFlowX.value>0)
//						{
//							pidFlowX.value-=(speed/(float)speedup_step);
//							if(pidFlowX.value<0)
//							{
//								pidFlowX.value=0;
//							}				
//						}
//						else if(pidFlowX.value<0)
//						{
//							pidFlowX.value+=(speed/(float)speedup_step);
//							if(pidFlowX.value>0)
//							{
//								pidFlowX.value=0;
//							}
//						}			
//				
//				}//0�Ĵ���ʽ
////				pidFlowX.dOut = OpticalFlow_xKd*(FlowxErro-Last_FlowX)/OuterPID_dt;
////				pidFlowX.dOut = Limits_data(pidFlowX.dOut,0.02f,-0.02f);
////				pidFlowX.dOut = 0;
////				pidFlowX.value = pidFlowX.value+pidFlowX.dOut;
////				Last_FlowX = RT_Info.FlowX;
//				
//				//Y
//				float FlowyErro = -RT_Info.FlowY;
////				
//				YspeedWeight = 1-(ABS(FlowyErro)/(float)60);

//				if(FlowyErro>Area)
//				{
//					if(pidFlowY.value<speed_h)
//					{
//						pidFlowY.value+=(speed_h/(float)speedup_step1);
//						if(pidFlowY.value>=speed_h)
//						{
//							pidFlowY.value=speed_h;						
//						}
//					}
//					else if(pidFlowY.value>speed_h)
//					{
//						pidFlowY.value-=(speed_h/(float)speedup_step1);
//						if(pidFlowY.value<=speed_h)
//						{
//							pidFlowY.value=speed_h;						
//						}					
//					}
//				}//���� [0.05,��]����ʽ
//				else if((FlowyErro<=Area)&&(FlowyErro>0))
//				{
//					if(pidFlowY.value>speed)
//					{
//						pidFlowY.value-=(speed/(float)speedup_step);
//						if(pidFlowY.value<speed)
//						{
//							pidFlowY.value=speed;
//						}
//					}
//					else if(pidFlowY.value<speed)
//					{
//						pidFlowY.value+=(speed/(float)speedup_step);
//						if(pidFlowY.value>speed)
//						{
//							pidFlowY.value=speed;
//						}
//					}	
//				}//���� [0,0.05]����ʽ		
//				else if((FlowyErro>=-Area)&&(FlowyErro<0))
//				{
//					if(pidFlowY.value>=-speed)
//					{
//						pidFlowY.value-=(speed/(float)speedup_step);
//						if(pidFlowY.value<=-speed)
//						{
//							pidFlowY.value=-speed;
//						}
//					}
//					else if(pidFlowY.value<-speed)
//					{
//						pidFlowY.value+=(speed/(float)speedup_step);
//						if(pidFlowY.value>-speed)
//						{
//							pidFlowY.value=-speed;
//						}
//					}	
//				}//���� [-0.05,0]����ʽ		
//				else if((FlowyErro<-Area))
//				{
//					if(pidFlowY.value>=-speed_h)
//					{
//						pidFlowY.value-=(speed_h/(float)speedup_step1);
//						if(pidFlowY.value<=-speed_h)
//						{
//							pidFlowY.value=-speed_h;
//						}
//					}
//					else if(pidFlowY.value<-speed_h)
//					{
//						pidFlowY.value+=(speed_h/(float)speedup_step1);
//						if(pidFlowY.value>-speed_h)
//						{
//							pidFlowY.value=-speed_h;
//						}
//					}	
//				}//���� [-��,-0.05]����ʽ	
//				else
//				{
//					if(pidFlowY.value>0)
//						{
//							pidFlowY.value-=(speed/(float)speedup_step);
//							if(pidFlowY.value<0)
//							{
//								pidFlowY.value=0;
//							}				
//						}
//						else if(pidFlowY.value<0)
//						{
//							pidFlowY.value+=(speed/(float)speedup_step);
//							if(pidFlowY.value>0)
//							{
//								pidFlowY.value=0;
//							}
//						}			
//				
//				}//0�Ĵ���ʽ
//				
////				pidFlowY.dOut = OpticalFlow_yKd*(FlowyErro-Last_FlowY)/OuterPID_dt;
////				pidFlowY.dOut = Limits_data(pidFlowY.dOut,0.02f,-0.02f);
////				pidFlowY.dOut = 0;
////				pidFlowY.value = pidFlowY.value + 0.01f;	
////				Last_FlowY = RT_Info.FlowY;
//			}
//		}
//		else 
//		{
//			pidFlowX.value=0.f;
//			pidFlowY.value=0.f;
//		}
//	}
}



