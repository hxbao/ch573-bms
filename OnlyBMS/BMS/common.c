/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"



#define NTC_RREF (10)
#define VDD_VAL (3300)

#if(MCU_LIB_SELECT ==1)
uint8_t CommonRam[512];
uint8_t CommonRam2[256];
#elif(MCU_LIB_SELECT ==2)
__attribute__((aligned(8))) uint8_t CommonRam[256];
__attribute__((aligned(8))) uint8_t CommonRam2[256];
#endif

//-40 ~110
const float NTC103AT[151] = {
	206.1, 194.6, 183.9, 173.8, 164.4, 155.5, 147.1, 139.3, 131.9, 125.0,//-40
	118.4, 112.3, 106.5, 101.0, 95.85, 91.00, 86.43, 82.12, 78.05, 74.20,//-30
	70.58, 67.16, 63.93, 60.87, 57.98, 55.24, 52.65, 50.20, 47.87, 45.66,//-20
	43.56, 41.58, 39.69, 37.90, 36.20, 34.58, 33.05, 31.59, 30.21, 28.89,//-10
	27.63, 26.42, 25.27, 24.18, 23.15, 22.17, 21.25, 20.36, 19.53, 18.73,//0
	17.97, 17.25, 16.56, 15.91, 15.28, 14.69, 14.12, 13.57, 13.05, 12.56,//10
	12.08, 11.63, 11.20, 10.78, 10.38, 10.00, 9.635, 9.285, 8.949, 8.627,//20
	8.319, 8.022, 7.738, 7.465, 7.203, 6.952, 6.710, 6.478, 6.255, 6.040,//30
	5.834, 5.636, 5.446, 5.262, 5.086, 4.917, 4.753, 4.596, 4.445, 4.300,//40
	4.160, 4.025, 3.895, 3.770, 3.649, 3.533, 3.421, 3.313, 3.210, 3.109,//50
	3.013, 2.920, 2.830, 2.743, 2.660, 2.579, 2.502, 2.427, 2.354, 2.285,//60
	2.217, 2.152, 2.089, 2.029, 1.970, 1.914, 1.859, 1.807, 1.756, 1.707,//70
	1.659, 1.613, 1.569, 1.526, 1.484, 1.451, 1.406, 1.369, 1.333, 1.299,//80
	1.265, 1.232, 1.201, 1.170, 1.140, 1.111, 1.083, 1.056, 1.030, 1.004,//90
	0.9788, 0.9546, 0.9311, 0.9082, 0.8860, 0.8644, 0.8434, 0.8229, 0.8031, 0.7838,//100
	0.7650};//110



	
uint16_t CalcuTemp(uint16_t Vntc)
{
	int16_t i;
	float tempcalcu, temperature;
	uint8_t ucTempeMiddle = 75;

	tempcalcu = (float)Vntc * NTC_RREF / (VDD_VAL - Vntc);

	if (tempcalcu >= NTC103AT[0]) //Determine whether the excess temperature resistance range
	{
		temperature = 2731 - 400;
	}
	else if (tempcalcu <= NTC103AT[150])
	{
		temperature = 2731 + 1100;
	}
	else
	{
		i = ucTempeMiddle;
		if (tempcalcu > NTC103AT[i])
		{
			for (i = ucTempeMiddle - 1; i >= 0; i--)
			{
				if (tempcalcu <= NTC103AT[i]) //NTC103AT[i+1]<resis<NTC103AT[i]
				{
					break;
				}
			}
			i++;
		}
		else
		{
			for (i = ucTempeMiddle + 1; i < 151; i++)
			{
				if (tempcalcu > NTC103AT[i]) //NTC103AT[i-1]<resis<NTC103AT[i]
				{
					break;
				}
			}
			i--;
		}
		ucTempeMiddle = i;

		temperature = (uint16_t)((ucTempeMiddle - 40) * 10 + (NTC103AT[i] - tempcalcu) * 10 / (NTC103AT[i] - NTC103AT[i + 1]) + 2731);
	}
	return (uint16_t)temperature;
}

uint16_t CalcuTemp1(float Rntc)
{
	int16_t i;
	float tempcalcu, temperature;
	uint8_t ucTempeMiddle = 81;

	tempcalcu = Rntc;

	if (tempcalcu >= NTC103AT[0]) //Determine whether the excess temperature resistance range
	{
		temperature = 2731 - 400;
	}
	else if (tempcalcu <= NTC103AT[150])
	{
		temperature = 2731 + 1100;
	}
	else
	{
		i = ucTempeMiddle;
		if (tempcalcu > NTC103AT[i])
		{
			for (i = ucTempeMiddle - 1; i >= 0; i--)
			{
				if (tempcalcu <= NTC103AT[i]) //NTC103AT[i+1]<resis<NTC103AT[i]
				{

					break;
				}
			}
			i++;
		}
		else
		{
			for (i = ucTempeMiddle + 1; i < 151; i++)
			{
				if (tempcalcu > NTC103AT[i]) //NTC103AT[i-1]<resis<NTC103AT[i]
				{
					break;
				}
			}
			i--;
		}
		ucTempeMiddle = i;

		temperature = (uint16_t)((ucTempeMiddle - 40) * 10 + (NTC103AT[i] - tempcalcu) * 10 / (NTC103AT[i] - NTC103AT[i + 1]) + 2731);
	}
	return (uint16_t)temperature;
}

//输入温度，反查到ntc电阻值
float CalaRntcFromTemp(uint16_t temp)
{
	temp = temp - (2731 - 400); //-40-110

	if (temp <= 1500)
	{
		return 1000 * NTC103AT[temp / 10]; //0~150
	}
	else
	{
		return 0;
	}
}

void MyMemcpy(uint8_t *dst,uint8_t *src,uint16_t len)
{
	uint16_t i;
	for(i = 0;i<len;i++)
	{
		*(dst+i) = *(src+i);
	}
}
