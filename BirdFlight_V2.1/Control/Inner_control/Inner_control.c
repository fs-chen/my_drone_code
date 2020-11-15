/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * ���� 	:Xiluna Tech
 * �ļ��� :Inner_control.c
 * ����   :�ڻ����ƺ���
 * ����   :http://xiluna.com/
 * ���ں� :XilunaTech
**********************************************************************************/
#include "Inner_control.h"

//�ֲ�����
PIDOut  pidRatePitch,pidRateRoll,pidRateYaw,
					pidAccHeight,pidAccPitch,pidAccRoll,
						pidXSpeed,pidYSpeed,pidFlowx,pidFlowy;

static volatile float intergrateLimit = 80;
static volatile float rateErroY_History = 0;
static volatile float rateErroX_History = 0;
static volatile float rateErroZ_History = 0;
static volatile float lastratePErro = 0;
static volatile float lastrateRErro = 0;
static volatile float lastrateYErro = 0;
static volatile float heightVErr_History = 0;
static volatile float pointxVErr_History = 0;
static volatile float pointyVErr_History = 0;
static volatile float flowxVErr_History = 0;
static volatile float flowyVErr_History = 0;
static volatile float lastheightVDelta=0;
static volatile float lastpointVxDelta=0;
static volatile float lastpointVyDelta=0;
static volatile float lastflowVxDelta=0;
static volatile float	lastflowVyDelta=0;
void Calculate_output(void);

//��ͨ����
// low pass filter:           0.079577472903393 
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 2 Hz -> _filter = 79.5774e-3
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
volatile float lowpass_filter = 7.9577e-3;
extern uint8_t takeoff_suce_flag;
void Inner_pidinit()
{
	takeoff_suce_flag=0;//////////////////
	pidRatePitch.iOut = 0;
	pidRateRoll.iOut = 0;
	pidRateYaw.iOut = 0;
	pidFlowx.iOut=0;
	pidFlowy.iOut=0;
	pidXSpeed.iOut = 0;
	pidYSpeed.iOut = 0;
	pidAccHeight.iOut = 0;
	rateErroX_History = 0;
	rateErroY_History = 0;
	rateErroZ_History = 0;
	lastratePErro = 0;
	lastrateRErro = 0;
	lastrateYErro = 0;
	//�߶��ٶ�
	lastheightVDelta = 0;
	heightVErr_History = 0;
	//��λ���ٶ�
	lastpointVxDelta = 0;
	pointxVErr_History = 0;
	lastpointVyDelta = 0;
	pointyVErr_History = 0;
	//�����ٶ�
	flowxVErr_History = 0;
	lastflowVxDelta = 0;
	flowyVErr_History = 0;
	lastflowVyDelta = 0;
} 

extern float IdealValueA;
extern LineInspectObject LineInspect;
void AttitudeInner_control()
{
	static unsigned int AttitudeInnertPre=0;
	unsigned int AttitudeInnert;
	static float AttitudeInnerPID_dt;
	AttitudeInnert = micros();
  AttitudeInnerPID_dt = (AttitudeInnertPre>0)?((AttitudeInnert-AttitudeInnertPre)/1000000.0f):1;
  AttitudeInnertPre = AttitudeInnert;
	
	float deltaPitchRateErro,deltaRollRateErro,deltaYawRateErro;
	/************************ �ڻ�Pitch���ٶȻ�����************************/	
	//���Ҫ����Pitch�Ǵ���ڻ�ʵ�飬��������pidPitch.value����Ϊ��>	Target_Info.RatePitch
	float ratePitchErro;
//	if(FlightControl.droneMode!=Drone_Mode_RatePitch)
//	{
	ratePitchErro = (pidPitch.value - RT_Info.ratePitch);
//	}
//	else//˵��RdroneStudioѡ���˲����ڻ�Pitch,��������Ϊ�ֱ���������������
//	{ 
//		ratePitchErro = (Target_Info.RatePitch - RT_Info.ratePitch);
//	} 
//	
	deltaPitchRateErro = (ratePitchErro - rateErroY_History)/AttitudeInnerPID_dt;
	/*20Hz��ͨ�˲���*/
	deltaPitchRateErro = lastratePErro + (AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaPitchRateErro - lastratePErro);	
	lastratePErro=deltaPitchRateErro;
	
	pidRatePitch.pOut = Para_Info.ratePitch.Kp * ratePitchErro;
	pidRatePitch.dOut = Para_Info.ratePitch.Kd * deltaPitchRateErro;
	pidRatePitch.iOut += Para_Info.ratePitch.Ki * ratePitchErro;
	
	pidRatePitch.iOut = Limits_data(pidRatePitch.iOut,intergrateLimit,-intergrateLimit);
	
	rateErroY_History = ratePitchErro;
	
	pidRatePitch.value = pidRatePitch.pOut
														+ pidRatePitch.dOut
															   + pidRatePitch.iOut;
	/*�޷�Pitch�������*/
	pidRatePitch.value = Limits_data(pidRatePitch.value,300,-300);
	
	/************************ �ڻ�Roll���ٶȻ�����************************/	
	//���Ҫ����Roll�Ǵ���ڻ�ʵ�飬��������pidRoll.value����Ϊ��>	Target_Info.RateRoll
	float rateRollErro;
//	if(FlightControl.droneMode!=Drone_Mode_RateRoll)
//	{
	rateRollErro = (pidRoll.value- RT_Info.rateRoll); 
//	}	
//	else
//	{
//		rateRollErro = (Target_Info.RateRoll- RT_Info.rateRoll); 
//	}
	deltaRollRateErro = (rateRollErro - rateErroX_History)/AttitudeInnerPID_dt;
	
	deltaRollRateErro = lastrateRErro + 
   			(AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaRollRateErro - lastrateRErro);	
	lastrateRErro=deltaRollRateErro;

	pidRateRoll.pOut = Para_Info.rateRoll.Kp * rateRollErro;
	pidRateRoll.dOut = Para_Info.rateRoll.Kd * deltaRollRateErro;
	pidRateRoll.iOut += Para_Info.rateRoll.Ki * rateRollErro;
	/*�����޷�*/
	pidRateRoll.iOut = Limits_data(pidRateRoll.iOut,intergrateLimit,-intergrateLimit);
	
	rateErroX_History = rateRollErro;
	
	pidRateRoll.value = pidRateRoll.pOut
													+pidRateRoll.dOut
															+pidRateRoll.iOut;
	/*�޷�Roll�������*/
	pidRateRoll.value = Limits_data(pidRateRoll.value,300,-300);
	
	/************************ �ڻ�Yaw���ٶȻ�����************************/	
//	Control_Info.yV =0.f;
	float rateYawErro= (LineInspect.Target.Target_Yaw_Acc - RT_Info.rateYaw);
	deltaYawRateErro = (rateYawErro - rateErroZ_History)/AttitudeInnerPID_dt;
	
	deltaYawRateErro = lastrateYErro + 
   			(AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaYawRateErro - lastrateYErro);	
	lastrateYErro=deltaYawRateErro;

	pidRateYaw.pOut =  Para_Info.rateYaw.Kp * rateYawErro;
	pidRateYaw.dOut =  Para_Info.rateYaw.Kd * deltaYawRateErro;
	pidRateYaw.iOut += Para_Info.rateYaw.Ki * rateYawErro;
	/*�����޷�*/
	pidRateYaw.iOut = Limits_data(pidRateYaw.iOut,intergrateLimit,-intergrateLimit);	
	
	rateErroZ_History = rateYawErro;
	
	pidRateYaw.value=pidRateYaw.pOut
											+pidRateYaw.dOut
													+pidRateYaw.iOut;
	/*�޷�Yaw�������*/
	pidRateYaw.value = Limits_data(pidRateYaw.value,300,-300);	
	
	/**********************�ڻ�������������**********************/
	Calculate_output();
}
void PostionInner_control()
{
	//ʱ�����
	static unsigned int PosInnertPre=0;
	unsigned int PosInnert;
	static float PosInnerPID_dt;
	PosInnert=micros();
	//�ڻ����ڼ���
	PosInnerPID_dt = (PosInnertPre>0)?((PosInnert-PosInnertPre)/1000000.0f):1;
	//��ȫ�ĳ�
	PosInnertPre=PosInnert;
	
	/************************ �߶��ٶ��ڻ� ************************/
	//���Ҫ���Դ������ڻ�ʵ�飬��������pidHeight.value����Ϊ��>Target_Info.AccHeight
	float verro_us100 = pidHeight.value - RT_Info.US100_Alt_V;
	//�ڻ����������룬���⻷�ĸ߶�PID���ֵ����õ���
	//�ٶ���� = �߶���������� - ���������ٶȷ���
	float vdelta_us100 = (verro_us100 - heightVErr_History)/PosInnerPID_dt;
	//�����ٶ�΢��ֵ
	
	/*20Hz��ͨ�˲���*/
	vdelta_us100 = lastheightVDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_us100 - lastheightVDelta);
	//��ͨ�˲�����
	lastheightVDelta = vdelta_us100;
	//��¼��һ�ε���ֵ���˴������˲���
	heightVErr_History = verro_us100;
	//��ʷֵ��¼���˴�����������
	pidAccHeight.pOut = verro_us100  * Para_Info.accHeight.Kp;
	//P�����
	pidAccHeight.dOut = vdelta_us100 * Para_Info.accHeight.Kd;
	//D�����
	pidAccHeight.iOut += verro_us100 * Para_Info.accHeight.Ki;
	//I�����
	pidAccHeight.iOut = Limits_data(pidAccHeight.iOut,120,-120);	
	//I���޷�
	pidAccHeight.value = pidAccHeight.pOut
													+pidAccHeight.dOut
															+pidAccHeight.iOut
																	+ pidHeight.feedforwardOut;//����ǰ��У׼
	//�߶ȿ����ڻ��ٶȻ������
	pidAccHeight.value = Limits_data(pidAccHeight.value,300,-100);	
	//������޷���ǰ��+PID
	
///**********���²����Ƕ���������ݣ�����ʵ�ʵķ��������޸�***************/		


////	/************************ λ���ٶ��ڻ� ************************/
////	if(BlackspotsFlag == 1 && OpticalflowFlag == 0 && RT_Info.US100_Alt>0.1f)
////	{
////		//�ڵ�ʶ��
////		/************************ λ���ٶ��ڻ� ************************/
////		float pointvx_Kp = -4.0;
////		float pointvx_Ki = -0.001;
////		float pointvx_Kd = -0.30;
////		// X��λ���ٶ�
////		float verro_pointx = pidPointX.value - RT_Info.PointX_V;
////		//Xλ���ڻ��ٶ���� = λ���⻷��� - ���ٶȷ���
////		//
////		float vdelta_pointx = (verro_pointx - pointxVErr_History)/PosInnerPID_dt;
////		//�ٶ�΢��
////		
////		/*20Hz��ͨ�˲���*/
////		vdelta_pointx = lastpointVxDelta + 
////   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_pointx - lastpointVxDelta);
////		//��ͨ�˲���
////		lastpointVxDelta = vdelta_pointx;
////		//��¼��һ�ε㣬�����˲���
////		pointxVErr_History = verro_pointx;
////		//��¼��һ�ε㣬���ڼ������
////		pidXSpeed.pOut = verro_pointx  * pointvx_Kp;
////		pidXSpeed.dOut = vdelta_pointx * pointvx_Kd;
////		pidXSpeed.iOut += verro_pointx * pointvx_Ki;
////		//PID����
////		pidXSpeed.iOut = Limits_data(pidXSpeed.iOut,4,-4);	
////		//�����޷�
////		pidXSpeed.value = pidXSpeed.pOut
////														+pidXSpeed.dOut
////																	+pidXSpeed.iOut;
////																			 //+pidPointX.feedforwardOut;//����ǰ��У׼
////		//PID���
////		pidXSpeed.value = Limits_data(pidXSpeed.value,10,-10);	
////		//����޷�
////		Target_Info.Roll = pidXSpeed.value;
////		//��X��λ�ÿ���ת����ŷ���ǵ������Ƕ�
////		float pointvy_Kp = -4.0;
////		float pointvy_Ki = -0.001;
////		float pointvy_Kd = -0.30;
////		// Y��λ���ٶ�
////		float verro_pointy = pidPointY.value - RT_Info.PointY_V;
////		float vdelta_pointy = (verro_pointy - pointyVErr_History)/PosInnerPID_dt;
////		/*20Hz��ͨ�˲���*/
////		vdelta_pointy = lastpointVyDelta + 
////   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_pointy - lastpointVyDelta);
////		
////		lastpointVyDelta = vdelta_pointy;
////		pointyVErr_History = verro_pointy;

////		pidYSpeed.pOut = verro_pointy  * pointvy_Kp;
////		pidYSpeed.dOut = vdelta_pointy * pointvy_Kd;
////		pidYSpeed.iOut += verro_pointy * pointvy_Ki;

////		pidYSpeed.iOut = Limits_data(pidYSpeed.iOut,4,-4);	

////		pidYSpeed.value = pidYSpeed.pOut
////														+pidYSpeed.dOut
////																	+pidYSpeed.iOut;
////																			//+ pidPointY.feedforwardOut;//����ǰ��У׼

////		pidYSpeed.value = Limits_data(pidYSpeed.value,10,-10);	

////		Target_Info.Pitch = pidYSpeed.value;
////	}
//	/************************ �����ٶȿ��� ************************/
//	//����ֻ�е�����û��λ�û���ֻ�нǶȻ�
//	
////	if(BlackspotsFlag == 0 && OpticalflowFlag == 1 && RT_Info.US100_Alt>0.1f)
////	{
//		//����ģʽ
//		/***************X��PID����***************/
//		// ����pitch ������Y
//		// ����roll �Ǹ���X
//		
//		float flowvx_Kp = -9.75f;
//		float flowvx_Ki = -0.003f;
//		float flowvx_Kd = -0.105f;
//		static float lastTarX=0;
////		/*X��λ���ٶȵ���*/
//		pidFlowX.value=0.f;
////		Control_Info.rV=0.f;
//		//��ʱ
//		
//		float verro_flowx=(float)(RT_Info.FlowX_V - LineInspect.Target.Target_X_V);//-0.005f*Control_Info.rV
//		
//		float vdelta_flowx=(verro_flowx-flowxVErr_History)/PosInnerPID_dt;
//		/*20Hz��ͨ�˲���*/
//		vdelta_flowx = lastflowVxDelta + 
//					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowx - lastflowVxDelta);

//		lastflowVxDelta = vdelta_flowx;
//		flowxVErr_History = verro_flowx;

//		pidFlowx.pOut=flowvx_Kp * verro_flowx;
//		pidFlowx.dOut=flowvx_Kd * vdelta_flowx;
//		
////		//���ַ���
////		if(ABS(verro_flowx) <= 0.2f)
//		{
//			pidFlowx.iOut+=flowvx_Ki * verro_flowx;
//		}
//		
//		pidFlowx.iOut = Limits_data(pidFlowx.iOut,2,-2);
//		//integrated limit
//		pidFlowx.dOut=Limits_data(pidFlowx.dOut,3,-3);
//		//different limit
//		pidFlowx.value = pidFlowx.pOut
//												+pidFlowx.dOut
//														+pidFlowx.iOut;
//		
//		pidFlowx.value = Limits_data(pidFlowx.value,10,-10);
//		
//		Target_Info.Roll = pidFlowx.value;
//		Target_Info.Roll = 0.5f*Target_Info.Roll + 0.5f*lastTarX;
//		lastTarX=Target_Info.Roll;
//		
//		/***************Y��PID����***************/			
//		float flowvy_Kp = -9.75f;
//		float flowvy_Ki = -0.003f;
//		float flowvy_Kd = -0.105f;
//		static float lastTarY=0;
//		/*X��λ���ٶȵ���*/
//		pidFlowY.value=0.f;
////		Control_Info.pV=0.f;
//		//��ʱ
//		float verro_flowy=(float)(RT_Info.FlowY_V - LineInspect.Target.Target_Y_V);//-0.005f*Control_Info.pV
//		float vdelta_flowy=(verro_flowy-flowyVErr_History)/PosInnerPID_dt;
//		/*20Hz��ͨ�˲���*/
//		vdelta_flowy = lastflowVyDelta + 
//					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowy - lastflowVyDelta);

//		lastflowVyDelta = vdelta_flowy;
//		flowyVErr_History = verro_flowy;
//		pidFlowy.pOut=flowvy_Kp * verro_flowy;
//		pidFlowy.dOut=flowvy_Kd * vdelta_flowy;
//		
////		//���ַ���
////		if(ABS(verro_flowy) <= 0.2f)
//		{
//			pidFlowy.iOut+=flowvy_Ki * verro_flowy;
//		}

//		pidFlowy.iOut = Limits_data(pidFlowy.iOut,2,-2);
//		pidFlowy.dOut=Limits_data(pidFlowy.dOut,2,-2);
//		pidFlowy.value = pidFlowy.pOut
//												+pidFlowy.dOut
//														+pidFlowy.iOut; 
//														
//		pidFlowy.value = Limits_data(pidFlowy.value,10,-10);

//		Target_Info.Pitch = -pidFlowy.value;
//		Target_Info.Pitch = 0.5f*Target_Info.Pitch + 0.5f*lastTarY;
//		lastTarY = Target_Info.Pitch;
//		
////	} 
}

void Calculate_output()
{
	if(FlightControl.droneMode==Drone_Mode_4Axis)
	{										
			Throttle_Info.M1 =  - pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ 650;
		
			Throttle_Info.M2 =  + pidRatePitch.value 
													- pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ 650;
		
			Throttle_Info.M3 =  + pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value
													- pidRateYaw.value 
													+ 650;
		
			Throttle_Info.M4 =  - pidRatePitch.value 
													+ pidRateRoll.value 
													+ pidAccHeight.value 
													+ pidRateYaw.value 
													+ 650;
	}
//	else if(FlightControl.droneMode==Drone_Mode_Pitch || FlightControl.droneMode==Drone_Mode_RatePitch)
//	{
//			Throttle_Info.M1 = - pidRatePitch.value + throttleBasic;
//			Throttle_Info.M2 = + pidRatePitch.value + throttleBasic;
//			Throttle_Info.M3 = + pidRatePitch.value + throttleBasic;
//			Throttle_Info.M4 = - pidRatePitch.value + throttleBasic;
//	}
//	else if(FlightControl.droneMode==Drone_Mode_Roll || FlightControl.droneMode==Drone_Mode_RateRoll)
//	{
//			Throttle_Info.M1 = - pidRateRoll.value + throttleBasic;
//			Throttle_Info.M2 = - pidRateRoll.value + throttleBasic;
//			Throttle_Info.M3 = + pidRateRoll.value + throttleBasic;
//			Throttle_Info.M4 = + pidRateRoll.value + throttleBasic;
//	}
	
	if(Throttle_Info.M1 > 900)  Throttle_Info.M1=900;
	if(Throttle_Info.M2 > 900)  Throttle_Info.M2=900;
	if(Throttle_Info.M3 > 900)  Throttle_Info.M3=900;
	if(Throttle_Info.M4 > 900)  Throttle_Info.M4=900;
	
	if(Throttle_Info.M1 < 50)  Throttle_Info.M1=50;
	if(Throttle_Info.M2 < 50)  Throttle_Info.M2=50;
	if(Throttle_Info.M3 < 50)  Throttle_Info.M3=50;
	if(Throttle_Info.M4 < 50)  Throttle_Info.M4=50;
	
//	if(cnt<1000){PID_OUT(100,100,100,100);cnt++;}
	PID_OUT(Throttle_Info.M1,Throttle_Info.M2,Throttle_Info.M3,Throttle_Info.M4);
}
/*���ݲ�ͬʵ��ѡ��ͬ���ͨ��
* Motor1-4�����1-4��PWM���ռ�ձȣ���ΧΪ0-1000
* FlightMode������ģʽ��Ҳ����ʵ��ѡ��
*/
void PID_OUT(unsigned int Motor1,
						 unsigned int Motor2,
						 unsigned int Motor3,
						 unsigned int Motor4)
{
		Motor1+=1000;
		Motor2+=1000;
		Motor3+=1000;
		Motor4+=1000;
	
	  if(RT_Info.lowPowerFlag==1)
		{
			TIM2->CCR1=1000;
			TIM2->CCR2=1000;
			TIM2->CCR3=1000;
			TIM2->CCR4=1000;
		}
		else
		{
			TIM2->CCR1=Motor1;
			TIM2->CCR2=Motor2;
			TIM2->CCR3=Motor3;
			TIM2->CCR4=Motor4;
		}
}



