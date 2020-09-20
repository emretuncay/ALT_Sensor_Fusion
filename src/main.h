#define GNSS_vertical_speed_sensor_filter_coff    (float)0.98
#define fusined_vercital_speed_sensor_filter_coff (float)0.02

#define Barometer1_Channel 	   0
#define Barometer2_Channel 	   1
#define GNSS_Altimeter_Channel 2

Fusion_Sensor 				altitude;
Fusion_Sensor 				Vertical_Speed_Fusined;
Sensor 						GNSS_Vertical_Speed;
Complementary_Kalman		Vertical_Speed;

float B1[10] = {1.148981, 2.965024, 3.410747, 4.500512, 5.780990,
6.398805, 7.658461, 8.804929, 9.950192, 10.812473  };
float B2[10] = {1.216667, 2.781305, 3.990797, 4.838954, 5.880707,
6.794259, 7.716735, 8.827012, 9.647706, 10.905593 };
float B3[10] = {1.167923, 2.703743, 3.960820, 4.910017, 5.655322,
6.852176, 7.631121, 8.880161, 9.525737, 10.983303,  };
float Vspeed[10] = {1.867923, 2.703743, 3.960820, 4.910017, 5.655322,
6.852176, 7.631121, 8.880161, 9.525737, 10.983303  };

void Init();
