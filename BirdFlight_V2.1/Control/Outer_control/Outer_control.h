#ifndef __OUTER_CONTROL_H
#define __OUTER_CONTROL_H
#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "Neurons.h"

extern PIDOut pidPitch,pidRoll,pidYaw,pidHeight,pidPointX,pidPointY,pidFlowX,pidFlowY;
void Outer_pidinit(void);
void AttitudeOuter_control(void);
void PostionOuter_control(void);

#endif


