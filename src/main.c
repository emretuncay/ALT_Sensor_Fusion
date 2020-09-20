#include "DSP.h"
#include "main.h"
#define SampleFrequency 100

void Init(){
	altitude.FSensor[0].old_row_value = 0;
	altitude.FSensor[1].old_row_value = 0;
	altitude.FSensor[2].old_row_value = 0;

	altitude.Fusined_Value.row_value     = 0;
	altitude.Fusined_Value.old_row_value = 0;

	GNSS_Vertical_Speed.old_row_value = 0;

	Vertical_Speed.Augment_Value = 0;
	Vertical_Speed.Sensor_Value  = 0;
	Vertical_Speed.KALMAN.input              = 0;
	Vertical_Speed.KALMAN.error_covariance   = 1;
	Vertical_Speed.KALMAN.old_predict_output = 0;
	Vertical_Speed.KALMAN.predict_output     = 0;
	Vertical_Speed.KALMAN.KALMAN_gain        = 0;
}

int main(void) {

	Init();
	for(int i=0;i<10;i++){
		altitude.FSensor[Barometer1_Channel].row_value     = B1[i];
		altitude.FSensor[Barometer2_Channel].row_value     = B2[i];
		altitude.FSensor[GNSS_Altimeter_Channel].row_value = B3[i];

		//printf("B1 -> %f\n",altitude.FSensor[Barometer1_Channel].row_value);
		//printf("B2 -> %f\n",altitude.FSensor[Barometer2_Channel].row_value);
		//printf("B3 -> %f\n",altitude.FSensor[GNSS_Altimeter_Channel].row_value);

		Fusion(&altitude, 3);

		//printf("Fusined altitude -> %f\n",altitude.Fusined_Value.row_value);

		Vertical_Speed.Augment_Value = altitude_2_vertical_speed(&(altitude.Fusined_Value), SampleFrequency);
		//printf("Estimated Vertical Speed -> %f\n",Vertical_Speed.Augment_Value);

		Vertical_Speed.Sensor_Value = (float)Vspeed[i];
		//printf("GNSS Vertical Speed -> %f\n",Vertical_Speed.Sensor_Value);

		Complementary_KALMAN_Filter(&Vertical_Speed);
		//Vertical_Speed.Sensor_Value = Vertical_Speed.filtered_value;
		//Complementary_KALMAN_Filter(&Vertical_Speed);
		//printf("Filtered Vertical Speed -> %f\n",Vertical_Speed.filtered_value);
		//printf("/////////////////////////////////////////////////////////////////////\n");

		//printf("%f, ",altitude.Fusined_Value.row_value);
		printf("%f, ",Vertical_Speed.filtered_value);
	}

	while(1);

	return 0;
}
