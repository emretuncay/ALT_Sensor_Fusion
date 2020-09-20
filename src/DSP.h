#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define OK 0
#define NOT_OK -1
#define number_of_max_fusion_sensor 3

typedef struct _Sensor
{
	float row_value;
	float old_row_value;
	float fusion_weight;
}Sensor;

typedef struct _Fusion_Sensor
{
	Sensor FSensor[number_of_max_fusion_sensor];
	Sensor Fusined_Value;
}Fusion_Sensor;

typedef struct _KALMAN_Struct{
	float input;
	float predict_output;
	float old_predict_output;
	float KALMAN_gain;
	float error_covariance;
}KALMAN_Struct;

typedef struct _Complementary_Kalman{
	float Sensor_Value;
	float Augment_Value;
	KALMAN_Struct KALMAN;
	float filtered_value;
}Complementary_Kalman;

uint8_t update_weight(Fusion_Sensor* Sensor, uint8_t Number_of_Sensor);
void Fusion_core     (Fusion_Sensor* Sensor,uint8_t Number_of_Sensor);
void Fusion          (Fusion_Sensor* FuSensor,uint8_t Number_of_Sensor);

float altitude_2_vertical_speed(Sensor* Altitude_Sensor,float SampleFrequency);

uint8_t KALMAN_Filter(KALMAN_Struct* Kalman_Filter_Input);
uint8_t Complementary_KALMAN_Filter(Complementary_Kalman* Complementary_KALMAN_Filter_Input);
