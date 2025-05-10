#ifdef USING_VSCODE_AS_EDITOR
    #include "LowPass.h"
#endif


//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0


//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Select which gyroscope range to use (see the table below) - Default is 0
//	Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
//See the MPU6000 Register Map for more information


#define ACCEL_RANGE 0 //Select which accelerometer range to use (see the table below) - Default is 0
//	Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
//See the MPU6000 Register Map for more information



// Gyroscope R,P,Y: -0.374485,0.0190641,0.907492
// Accelerometer X,Y,Z: -10.4238,-13.5281,-16370.7


// Gyroscope R,P,Y: -375.061,19.4897,1203.41
// Accelerometer X,Y,Z: -10469.8,-13894.8,-3080.5
//Offsets - supply your own here (calculate offsets with getOffsets function)
//     Accelerometer
// #define A_OFF_X -11346.8
// #define A_OFF_Y -13575.3
// #define A_OFF_Z -3149.19
// //    Gyroscope
// #define G_OFF_X -375.061
// #define G_OFF_Y 19.4897
// #define G_OFF_Z 1203.41


/* filtro paso bajo */
// Gyroscope R,P,Y: -0.375197,0.0143413,1.285
// Accelerometer X,Y,Z: -10.466,-13.8993,-16370.7
//    Accelerometer
#define A_OFF_X -10.466
#define A_OFF_Y -13.8993
#define A_OFF_Z -16370.7
//    Gyroscope
#define G_OFF_X -0.375197
#define G_OFF_Y 0.0143413
#define G_OFF_Z 1.285




// Gyroscope R,P,Y: -375.02,11.09,1289.22
// Accelerometer X,Y,Z: 6927.75,-14281.5,-18293.4
//     Accelerometer
// #define A_OFF_X 6927.75
// #define A_OFF_Y -14281.5
// #define A_OFF_Z -18293.4
// //    Gyroscope
// #define G_OFF_X -375.02
// #define G_OFF_Y 11.09
// #define G_OFF_Z 1289.22


/* GYRO_RANGE 1 ACC_RANGE 2 */
// Gyroscope R,P,Y: -185.694,14.005,615.456
// Accelerometer X,Y,Z: -2781.72,-3548.17,-782.382
//    Accelerometer
// #define A_OFF_X -2781.72
// #define A_OFF_Y -3548.17
// #define A_OFF_Z -782.382
// //    Gyroscope
// #define G_OFF_X -185.694
// #define G_OFF_Y 14.005
// #define G_OFF_Z 615.456

/* GYRO_RANGE 1 ACC_RANGE 1 */
// Gyroscope R,P,Y: -188.571,16.636,853.799
// Accelerometer X,Y,Z: -5312.9,-6736.51,-1558.36
//    Accelerometer
// #define A_OFF_X -5312.9
// #define A_OFF_Y -6736.51
// #define A_OFF_Z -1558.36
// //    Gyroscope
// #define G_OFF_X -188.571
// #define G_OFF_Y 16.636
// #define G_OFF_Z 853.799


//-----------------------END MODIFY THESE PARAMETERS-----------------------

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <cmath>
#include <thread>

#define _POSIX_C_SOURCE 200809L //Used for calculating time

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0
#endif
#undef GYRO_RANGE


#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0
#endif
#undef ACCEL_RANGE




class MPU6050 {
	private:
		void _update();

		float _accel_angle[3];
		float _gyro_angle[3];
		float _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		float ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()

		int MPU6050_addr;
		int f_dev; //Device file

		float dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter


		float filterAlpha = 1.0f;
		pampas::LowPass<float> gyroFilters[3];
		pampas::LowPass<float> accFilters[3];
		
	public:
		MPU6050(int8_t addr);
		MPU6050(int8_t addr, bool run_update_thread);
		void getAccelRaw(float *x, float *y, float *z);
		void getGyroRaw(float *roll, float *pitch, float *yaw);
		void getAccel(float *x, float *y, float *z);
		void getGyro(float *roll, float *pitch, float *yaw);
		void getOffsets(float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off);
		int getAngle(int axis, float *result);
		bool calc_yaw;
};