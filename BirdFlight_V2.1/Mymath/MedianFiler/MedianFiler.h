#ifndef __MEDIANFILER_H
#define __MEDIANFILER_H

#include "stm32f4xx.h"


float Median_filter(int data,int measureNum,int *Filterdata);
float Median_filter_float(float data,int measureNum,float *Filterdata);

#endif 

