#ifndef __POSITION_H
#define __POSITION_H


#include "stm32f4xx.h"
#include "InertialFilter.h"
#include "IMU_AHRS.h"
#include "DronePara.h"
#include "Task.h"
#include "Timer5_Timing.h"
#include "digital_filter.h"
#include <os.h>
#ifndef CONSTANTS_ONE_G
#define CONSTANTS_ONE_G					8.80665f		/* m/s^2*/
#endif
void Filter_FIRinit(void);
void Position_Estimation(float Ultrasonic,float Xvision,float Yvision,float *Accel);
void OpticalFlow_Estimation(float Ultrasonic,float flow_x,float flow_y,float *Accz,float Accx,float Accy);
void h_Estimation(float Ultrasonic,float *Accz);
#endif 

