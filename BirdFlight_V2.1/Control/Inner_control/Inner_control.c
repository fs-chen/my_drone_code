/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Inner_control.c
 * 描述   :内环控制函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Inner_control.h"

//局部变量
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

//低通参数
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
	//高度速度
	lastheightVDelta = 0;
	heightVErr_History = 0;
	//点位置速度
	lastpointVxDelta = 0;
	pointxVErr_History = 0;
	lastpointVyDelta = 0;
	pointyVErr_History = 0;
	//光流速度
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
	/************************ 内环Pitch角速度环调节************************/	
	//如果要测试Pitch角打舵内环实验，把期望（pidPitch.value）改为—>	Target_Info.RatePitch
	float ratePitchErro;
//	if(FlightControl.droneMode!=Drone_Mode_RatePitch)
//	{
	ratePitchErro = (pidPitch.value - RT_Info.ratePitch);
//	}
//	else//说明RdroneStudio选择了测试内环Pitch,将期望改为手柄发送下来的期望
//	{ 
//		ratePitchErro = (Target_Info.RatePitch - RT_Info.ratePitch);
//	} 
//	
	deltaPitchRateErro = (ratePitchErro - rateErroY_History)/AttitudeInnerPID_dt;
	/*20Hz低通滤波器*/
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
	/*限幅Pitch最终输出*/
	pidRatePitch.value = Limits_data(pidRatePitch.value,300,-300);
	
	/************************ 内环Roll角速度环调节************************/	
	//如果要测试Roll角打舵内环实验，把期望（pidRoll.value）改为—>	Target_Info.RateRoll
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
	/*积分限幅*/
	pidRateRoll.iOut = Limits_data(pidRateRoll.iOut,intergrateLimit,-intergrateLimit);
	
	rateErroX_History = rateRollErro;
	
	pidRateRoll.value = pidRateRoll.pOut
													+pidRateRoll.dOut
															+pidRateRoll.iOut;
	/*限幅Roll最终输出*/
	pidRateRoll.value = Limits_data(pidRateRoll.value,300,-300);
	
	/************************ 内环Yaw角速度环调节************************/	
//	Control_Info.yV =0.f;
	float rateYawErro= (LineInspect.Target.Target_Yaw_Acc - RT_Info.rateYaw);
	deltaYawRateErro = (rateYawErro - rateErroZ_History)/AttitudeInnerPID_dt;
	
	deltaYawRateErro = lastrateYErro + 
   			(AttitudeInnerPID_dt / (lowpass_filter + AttitudeInnerPID_dt)) * (deltaYawRateErro - lastrateYErro);	
	lastrateYErro=deltaYawRateErro;

	pidRateYaw.pOut =  Para_Info.rateYaw.Kp * rateYawErro;
	pidRateYaw.dOut =  Para_Info.rateYaw.Kd * deltaYawRateErro;
	pidRateYaw.iOut += Para_Info.rateYaw.Ki * rateYawErro;
	/*积分限幅*/
	pidRateYaw.iOut = Limits_data(pidRateYaw.iOut,intergrateLimit,-intergrateLimit);	
	
	rateErroZ_History = rateYawErro;
	
	pidRateYaw.value=pidRateYaw.pOut
											+pidRateYaw.dOut
													+pidRateYaw.iOut;
	/*限幅Yaw最终输出*/
	pidRateYaw.value = Limits_data(pidRateYaw.value,300,-300);	
	
	/**********************内环最终输出到电机**********************/
	Calculate_output();
}
void PostionInner_control()
{
	//时间计算
	static unsigned int PosInnertPre=0;
	unsigned int PosInnert;
	static float PosInnerPID_dt;
	PosInnert=micros();
	//内环周期计算
	PosInnerPID_dt = (PosInnertPre>0)?((PosInnert-PosInnertPre)/1000000.0f):1;
	//安全的除
	PosInnertPre=PosInnert;
	
	/************************ 高度速度内环 ************************/
	//如果要测试纯油门内环实验，把期望（pidHeight.value）改为—>Target_Info.AccHeight
	float verro_us100 = pidHeight.value - RT_Info.US100_Alt_V;
	//内环的期望输入，由外环的高度PID输出值计算得到。
	//速度误差 = 高度输出的期望 - 超声波的速度反馈
	float vdelta_us100 = (verro_us100 - heightVErr_History)/PosInnerPID_dt;
	//计算速度微分值
	
	/*20Hz低通滤波器*/
	vdelta_us100 = lastheightVDelta + 
   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_us100 - lastheightVDelta);
	//低通滤波计算
	lastheightVDelta = vdelta_us100;
	//记录上一次的数值，此处用作滤波。
	heightVErr_History = verro_us100;
	//历史值记录，此处用作误差计算
	pidAccHeight.pOut = verro_us100  * Para_Info.accHeight.Kp;
	//P环输出
	pidAccHeight.dOut = vdelta_us100 * Para_Info.accHeight.Kd;
	//D环输出
	pidAccHeight.iOut += verro_us100 * Para_Info.accHeight.Ki;
	//I环输出
	pidAccHeight.iOut = Limits_data(pidAccHeight.iOut,120,-120);	
	//I环限幅
	pidAccHeight.value = pidAccHeight.pOut
													+pidAccHeight.dOut
															+pidAccHeight.iOut
																	+ pidHeight.feedforwardOut;//加入前馈校准
	//高度控制内环速度环总输出
	pidAccHeight.value = Limits_data(pidAccHeight.value,300,-100);	
	//总输出限幅：前馈+PID
	
///**********以下部分是定点控制内容，根据实际的方案做出修改***************/		


////	/************************ 位置速度内环 ************************/
////	if(BlackspotsFlag == 1 && OpticalflowFlag == 0 && RT_Info.US100_Alt>0.1f)
////	{
////		//黑点识别
////		/************************ 位置速度内环 ************************/
////		float pointvx_Kp = -4.0;
////		float pointvx_Ki = -0.001;
////		float pointvx_Kd = -0.30;
////		// X轴位置速度
////		float verro_pointx = pidPointX.value - RT_Info.PointX_V;
////		//X位置内环速度误差 = 位置外环输出 - 点速度反馈
////		//
////		float vdelta_pointx = (verro_pointx - pointxVErr_History)/PosInnerPID_dt;
////		//速度微分
////		
////		/*20Hz低通滤波器*/
////		vdelta_pointx = lastpointVxDelta + 
////   			(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_pointx - lastpointVxDelta);
////		//低通滤波器
////		lastpointVxDelta = vdelta_pointx;
////		//记录上一次点，用于滤波器
////		pointxVErr_History = verro_pointx;
////		//记录上一次点，用于计算误差
////		pidXSpeed.pOut = verro_pointx  * pointvx_Kp;
////		pidXSpeed.dOut = vdelta_pointx * pointvx_Kd;
////		pidXSpeed.iOut += verro_pointx * pointvx_Ki;
////		//PID计算
////		pidXSpeed.iOut = Limits_data(pidXSpeed.iOut,4,-4);	
////		//积分限幅
////		pidXSpeed.value = pidXSpeed.pOut
////														+pidXSpeed.dOut
////																	+pidXSpeed.iOut;
////																			 //+pidPointX.feedforwardOut;//加入前馈校准
////		//PID输出
////		pidXSpeed.value = Limits_data(pidXSpeed.value,10,-10);	
////		//输出限幅
////		Target_Info.Roll = pidXSpeed.value;
////		//将X轴位置控制转换成欧拉角的期望角度
////		float pointvy_Kp = -4.0;
////		float pointvy_Ki = -0.001;
////		float pointvy_Kd = -0.30;
////		// Y轴位置速度
////		float verro_pointy = pidPointY.value - RT_Info.PointY_V;
////		float vdelta_pointy = (verro_pointy - pointyVErr_History)/PosInnerPID_dt;
////		/*20Hz低通滤波器*/
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
////																			//+ pidPointY.feedforwardOut;//加入前馈校准

////		pidYSpeed.value = Limits_data(pidYSpeed.value,10,-10);	

////		Target_Info.Pitch = pidYSpeed.value;
////	}
//	/************************ 光流速度控制 ************************/
//	//光流只有单环，没有位置环，只有角度环
//	
////	if(BlackspotsFlag == 0 && OpticalflowFlag == 1 && RT_Info.US100_Alt>0.1f)
////	{
//		//光流模式
//		/***************X轴PID调节***************/
//		// 负的pitch 是正的Y
//		// 负的roll 是负的X
//		
//		float flowvx_Kp = -9.75f;
//		float flowvx_Ki = -0.003f;
//		float flowvx_Kd = -0.105f;
//		static float lastTarX=0;
////		/*X轴位移速度调整*/
//		pidFlowX.value=0.f;
////		Control_Info.rV=0.f;
//		//暂时
//		
//		float verro_flowx=(float)(RT_Info.FlowX_V - LineInspect.Target.Target_X_V);//-0.005f*Control_Info.rV
//		
//		float vdelta_flowx=(verro_flowx-flowxVErr_History)/PosInnerPID_dt;
//		/*20Hz低通滤波器*/
//		vdelta_flowx = lastflowVxDelta + 
//					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowx - lastflowVxDelta);

//		lastflowVxDelta = vdelta_flowx;
//		flowxVErr_History = verro_flowx;

//		pidFlowx.pOut=flowvx_Kp * verro_flowx;
//		pidFlowx.dOut=flowvx_Kd * vdelta_flowx;
//		
////		//积分分离
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
//		/***************Y轴PID调节***************/			
//		float flowvy_Kp = -9.75f;
//		float flowvy_Ki = -0.003f;
//		float flowvy_Kd = -0.105f;
//		static float lastTarY=0;
//		/*X轴位移速度调整*/
//		pidFlowY.value=0.f;
////		Control_Info.pV=0.f;
//		//暂时
//		float verro_flowy=(float)(RT_Info.FlowY_V - LineInspect.Target.Target_Y_V);//-0.005f*Control_Info.pV
//		float vdelta_flowy=(verro_flowy-flowyVErr_History)/PosInnerPID_dt;
//		/*20Hz低通滤波器*/
//		vdelta_flowy = lastflowVyDelta + 
//					(PosInnerPID_dt / (lowpass_filter + PosInnerPID_dt)) * (vdelta_flowy - lastflowVyDelta);

//		lastflowVyDelta = vdelta_flowy;
//		flowyVErr_History = verro_flowy;
//		pidFlowy.pOut=flowvy_Kp * verro_flowy;
//		pidFlowy.dOut=flowvy_Kd * vdelta_flowy;
//		
////		//积分分离
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
/*根据不同实验选择不同输出通道
* Motor1-4：电机1-4的PWM输出占空比，范围为0-1000
* FlightMode：飞行模式，也就是实验选择
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



