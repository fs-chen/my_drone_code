/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :IMU_AHRS.c
 * 描述   :IMU原始数据读取及数据融合
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "IMU_AHRS.h"
#include "MPU6500.h"
#include "LSM303D.h"
#include "explore_systick.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include <math.h>
#include "Task.h"

// asin函数
float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0f;
	}
	if (v >= 1.0f) {
		return PI/2;
	}
	if (v <= -1.0f) {
		return -PI/2;
	}
	return asin(v);
}

void IMU_HardwareInit()
{
//	LSM303_Initial();           //lsm303寄存器配置
//	delay_ms(100);
	MPU6500_initialize();					//mpu6500寄存器配置
	delay_ms(100);
	MPU6500_Init_Offset();    		//6500初始化校准（平均滤波）
	delay_ms(100);
}

void IMU_getValues(float * values) 
{  
	int16_t accgyroval[9];
	int i;
	//读取磁力计的原始值
	LSM303_readMag(&accgyroval[6]);	
	//读取加速度计和陀螺仪的原始值
	MPU6500_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
	for(i = 0; i<6; i++) 
	{
    if(i < 3) 
		{
  			values[i] = (float) accgyroval[i];
    }
		//陀螺仪单位转化
    else 
		{
        values[i] = ((float) accgyroval[i]) / 16.4f;
    }
  }	
	values[6] = (float)accgyroval[6];
	values[7] = (float)accgyroval[7];
	values[8] = (float)accgyroval[8];
}

volatile float RDrone_R[3][3];
volatile float RDrone_R2[3][3];
volatile float Acc_Flow_x;
volatile float Acc_Flow_y;
volatile float Acc_Flow_z;
float Accel_Src[3];
void IMU_getInfo()
{
	static float q[4];
	static float getValue[9];
	static float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	IMU_getValues(getValue);										//获取原始数据
	MahonyAHRSupdateIMU(getValue[3] * PI/180, getValue[4] * PI/180, getValue[5] * PI/180,
  getValue[0], getValue[1], getValue[2]);			//Mahony算法融合数据
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
	
	//使用矩阵的时候可以快速使用
	q0q0 = q[0]*q[0];
	q0q1 = q[0]*q[1];
	q0q2 = q[0]*q[2];
	q0q3 = q[0]*q[3];
	q1q1 = q[1]*q[1];
	q1q2 = q[1]*q[2];
	q1q3 = q[1]*q[3];
	q2q2 = q[2]*q[2];
	q2q3 = q[2]*q[3];
	q3q3 = q[3]*q[3]; 
	//数据融合使用的矩阵
	RDrone_R2[0][0] = 2*(q0q0+q1q1)-1;
	RDrone_R2[0][1] = 2*(q1q2-q0q3);
	RDrone_R2[0][2] = 2*(q1q3+q0q2);
	RDrone_R2[1][0] = 2*(q1q2+q0q3);		
	RDrone_R2[1][1] = 2*(q0q0+q2q2)-1;
	RDrone_R2[1][2] = 2*(q2q3-q0q1);
	RDrone_R2[2][0] = 2*(q1q3-q0q2);
	RDrone_R2[2][1] = 2*(q2q3+q0q1);
	RDrone_R2[2][2] = 2*(q0q0+q3q3)-1;
	
	RDrone_R[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
	RDrone_R[0][1] = 2.f * (q1q2 + q0q3);
	RDrone_R[0][2] = 2.f * (q1q3 - q0q2);
	RDrone_R[1][0] = 2.f * (q1q2 - q0q3);		
	RDrone_R[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
	RDrone_R[1][2] = 2.f * (q2q3 + q0q1);
	RDrone_R[2][0] = 2.f * (q1q3 + q0q2);
	RDrone_R[2][1] = 2.f * (q2q3 - q0q1);
	RDrone_R[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
	//加速度数据
	Accel_Src[0] = getValue[0] / 16384;
	Accel_Src[1] = getValue[1] / 16384;
	Accel_Src[2] = getValue[2] / 16384;

	//角速度数据
	RT_Info.rateRoll=getValue[3]*100; 
	RT_Info.ratePitch=getValue[4]*100;
	RT_Info.rateYaw=getValue[5]*100;
	//角度数据
	RT_Info.Roll = (atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),
                       1 - 2.0f*(q[1]*q[1] + q[2]*q[2])))* 180/PI;										 
	RT_Info.Pitch = -safe_asin(2.0f*(q[0]*q[2] - q[1]*q[3]))* 180/PI;
	//融合磁力计
	RT_Info.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/PI; // yaw
}




