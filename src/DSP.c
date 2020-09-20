#include "DSP.h"
#include <stdio.h>
/*
 * Function: update_weight
 * ----------------------------
 *   Returns the square of the largest of its two input values
 *
 *   n1: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
uint8_t update_weight(Fusion_Sensor* Input, uint8_t Number_of_Sensor){

	float sensor_mean_value = 0;
	float diff[number_of_max_fusion_sensor] = {0};
	float diff_sum = 0;
	if(Number_of_Sensor > number_of_max_fusion_sensor){
		return NOT_OK;
	}
	else{
		for(uint8_t i=0;i<Number_of_Sensor;i++){
			sensor_mean_value += Input->FSensor[i].row_value;
		}
		sensor_mean_value = sensor_mean_value/Number_of_Sensor;
		for(uint8_t j=0;j<Number_of_Sensor;j++){
			diff[j] = fabs(sensor_mean_value-Input->FSensor[j].row_value);
			diff_sum = diff_sum + diff[j];
		}
		for(uint8_t k=0;k<Number_of_Sensor;k++){
			if(diff_sum == 0){
				Input->FSensor[k].fusion_weight = ((float)1/(float)Number_of_Sensor);
			}
			else
				Input->FSensor[k].fusion_weight = diff[k]/diff_sum;;
		}
		return OK;
	}
}
/*
 * Function: Fusion_core
 * ----------------------------
 *   Returns the square of the largest of its two input values
 *
 *   n1: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
void Fusion_core(Fusion_Sensor* FuSensor,uint8_t Number_of_Sensor)
{
	//float buff = (float)0;
/*
	for(uint8_t i;i<Number_of_Sensor;i++){
		buff = buff + (FuSensor->FSensor[i].row_value)*(FuSensor->FSensor[i].fusion_weight);
		printf("%f\n",FuSensor->FSensor[i].row_value);
		printf("%f\n",buff);
	}*/
	FuSensor->Fusined_Value.old_row_value = FuSensor->Fusined_Value.row_value;
	FuSensor->Fusined_Value.row_value = ((FuSensor->FSensor[0].row_value)*(FuSensor->FSensor[0].fusion_weight)) +
										((FuSensor->FSensor[1].row_value)*(FuSensor->FSensor[1].fusion_weight)) +
										((FuSensor->FSensor[2].row_value)*(FuSensor->FSensor[2].fusion_weight));
	//printf("%f\n",buff);
}
/*
 * Function: Fusion
 * ----------------------------
 *   Returns the square of the largest of its two input values
 *
 *   n1: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
void Fusion(Fusion_Sensor* FuSensor,uint8_t Number_of_Sensor){
	update_weight(FuSensor, Number_of_Sensor);
	Fusion_core(FuSensor, Number_of_Sensor);
}
/*
 * Function: altitude_2_vertical_speed
 * ----------------------------
 *   Returns the square of the largest of its two input values
 *
 *   n1: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
float altitude_2_vertical_speed(Sensor* Altitude_Sensor,float SampleFrequency){
	float output;
	//printf("%f\n",Altitude_Sensor->row_value);
	//printf("%f\n",Altitude_Sensor->old_row_value);
	output = (Altitude_Sensor->row_value - Altitude_Sensor->old_row_value)/((float)1/(float)SampleFrequency);
	return output;
	//printf("%f\n",Altitude_Sensor->row_value);
	//Altitude_Sensor->old_row_value = Altitude_Sensor->row_value;
}
/*
 * Function: KALMAN_Filter
 * ----------------------------
 *   Returns the filtered value of Input
 *
 *   Kalman_Filter_Input: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
uint8_t KALMAN_Filter(KALMAN_Struct* Kalman_Filter_Input){
	Kalman_Filter_Input->old_predict_output = Kalman_Filter_Input->predict_output;
	if(Kalman_Filter_Input->KALMAN_gain == -0.1)
		return NOT_OK;
	Kalman_Filter_Input->KALMAN_gain        = Kalman_Filter_Input->error_covariance/(Kalman_Filter_Input->error_covariance + (float)0.1);
	Kalman_Filter_Input->predict_output     = Kalman_Filter_Input->old_predict_output +
			Kalman_Filter_Input->KALMAN_gain*(Kalman_Filter_Input->input - Kalman_Filter_Input->old_predict_output);
	Kalman_Filter_Input->error_covariance = ((float)1 - Kalman_Filter_Input->KALMAN_gain)*Kalman_Filter_Input->error_covariance;
	return OK;
}
/*
 * Function: Complementary_KALMAN_Filter
 * ----------------------------
 *   Returns the square of the largest of its two input values
 *
 *   n1: one real value
 *   n2: the other real value
 *
 *   returns: the square of the larger of n1 and n2
 */
uint8_t Complementary_KALMAN_Filter(Complementary_Kalman* Complementary_KALMAN_Filter_Input){
	uint8_t status;
	Complementary_KALMAN_Filter_Input->KALMAN.input = Complementary_KALMAN_Filter_Input->Augment_Value-Complementary_KALMAN_Filter_Input->Sensor_Value;
	status = KALMAN_Filter(&(Complementary_KALMAN_Filter_Input->KALMAN));
	if(status == NOT_OK)
		return NOT_OK;
	Complementary_KALMAN_Filter_Input->filtered_value = Complementary_KALMAN_Filter_Input->Sensor_Value +
														Complementary_KALMAN_Filter_Input->KALMAN.predict_output;
	return OK;
}
