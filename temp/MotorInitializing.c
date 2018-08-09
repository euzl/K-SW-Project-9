#ifndef __MOTORINITIALIZING__
#define __MOTORINITIALIZING__

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include "IMU.c"
#include <signal.h>
#include <float.h>

#define DT 0.02			//loop period. 20ms
#define AA 0.97			//complementary filter constant
#define Relay_Ch1 25	//Relay that makes pan motor turns right	
#define Relay_Ch2 28	//Relay that makes pan motor turns left
#define Relay_Ch3 29	//Relay that makes tilt motor turns up
#define Relay_Ch4 24	//Relay that makes tilt motor turns down

#define A_GAIN 0.00573	//[deg/LSB]
#define G_GAIN 0.070	//[deg/s/LSB]
#define RAD_TO_DEG	57.29578	
#define M_PI 3.14159265358979323846

#define HORIZONTAL_DEGREE 0
#define SOUTH_DEGREE 180
#define NORTH_DEGREE 0

double headingArray[10] = {-DBL_MAX,};
	double calcAverage(){
		int i;
		int num=0;
		double sum=0;
		double min = DBL_MAX, max =-DBL_MAX;
		for(i=0; i< 10; i++) {
				if(headingArray[i] != DBL_MAX){
					if(headingArray[i] < min)
						min = headingArray[i];
					if(headingArray[i] > max)
						max = headingArray[i];
			}
		}

		for(i=0; i< 10; i++) {
			if(headingArray[i] != -DBL_MAX){
					if(max - min <= 270){
						num++;
						sum+=headingArray[i];
					}
					else{
							num++;
							if(headingArray[i] > 270){
								sum += headingArray[i]-360;
							}
							else{
								sum += headingArray[i];
							}
					}
			}
		}
		if(sum < 0)
				return (sum/num + 360);
		else
			return (sum/num);
	}
	
	void enqueueArray(double val){
		int idx=0;
		for(idx=0; idx< 10; idx++){
			if(headingArray[idx]==-DBL_MAX){
					headingArray[idx] = val;
					return;
			}
		}
		for(idx=0; idx<9; idx++) {
			headingArray[idx] = headingArray[idx+1];
		}
		headingArray[9] = val;
		return;
	}

	
	void INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
	{
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		signal(sig, SIG_IGN);
		exit(0);
	}

	int mymillis()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
	}

	int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
	{
		long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
		result->tv_sec = diff / 1000000;
		result->tv_usec = diff % 1000000;
		return (diff < 0);
	}

	int motorInitializing()
	{
		int i;
		for(i=0; i< 10; i++) {
			headingArray[i]=-DBL_MAX;
		}
		float rate_gyr_y = 0.0;		// [deg/sec]
		float rate_gyr_x = 0.0;		// [deg/sec]
		float rate_gyr_z = 0.0;		// [deg/sec]

		int accRaw[3];				//raw value of accelerometer (x, y, z)
		int gyrRaw[3]; 				//raw value of gyroscope (x, y, z)
		int magRaw[3];				//raw value of magnetometer (x, y, z)

		float gyroXangle = 0.0;
		float gyroYangle = 0.0;
		float gyroZangle = 0.0;
		float AccYangle = 0.0;
		float AccXangle = 0.0;
		float CFangleX = 0.0;
		float CFangleY = 0.0;
		//variables that is needed for compass
		float accXnorm, accYnorm, pitch, roll, magXcomp, magYcomp;

		int startInt = mymillis();
		struct timeval tvBegin, tvEnd, tvDiff;

		short int motor_status = 0; 	//0: stop, 1: right, 2: left, 3: up, 4: down

		signal(SIGINT, INThandler);

		detectIMU();				//IMU detect
		enableIMU();				//enabling IMU

		gettimeofday(&tvBegin, NULL);	//assign timevalue to tvBegin


		//Initializing the mortar's head (toward north & set horizentally)

		if (wiringPiSetup() == -1) return 0;
		pinMode(Relay_Ch1, OUTPUT);
		pinMode(Relay_Ch2, OUTPUT);
		pinMode(Relay_Ch3, OUTPUT);
		pinMode(Relay_Ch4, OUTPUT);

		printf("Setuping The RelayModule is [success]\n");
		//
		//tilt motor part
		startInt = mymillis();
		readACC(accRaw);		//read values from accelerometer to accRaw array
		readGYR(gyrRaw);		//read values from gyroscope to gryRaw array

		//convert Gyro raw to degrees per second
		rate_gyr_x = (float)gyrRaw[0] * G_GAIN;
		rate_gyr_y = (float)gyrRaw[1] * G_GAIN;
		rate_gyr_z = (float)gyrRaw[2] * G_GAIN;
		//Calculate the angles from the gyro
		gyroXangle += rate_gyr_x*DT;
		gyroYangle += rate_gyr_y*DT;
		gyroZangle += rate_gyr_z*DT;
		//Convert Accelerometer values to degrees
		AccXangle = (float)(atan2(accRaw[1], accRaw[2]) + M_PI)*RAD_TO_DEG;
		AccYangle = (float)(atan2(accRaw[2], accRaw[0]) + M_PI)*RAD_TO_DEG;
		//print angle values
		AccXangle -= (float)180.0;
		if (AccYangle > 90)
			AccYangle -= (float)270;
		else
			AccYangle += (float)90;
		printf("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m\n", gyroXangle, AccXangle, CFangleX, gyroYangle, AccYangle, CFangleY);
	
		//tilt until being horizon
		if (AccYangle > HORIZONTAL_DEGREE) {		//if AccYangle is lower than HORIZONTAL_DEGREE
			digitalWrite(Relay_Ch1, HIGH);
			digitalWrite(Relay_Ch2, HIGH);
			digitalWrite(Relay_Ch3, LOW);
			digitalWrite(Relay_Ch4, HIGH);
			motor_status = 3;
		}
		else if (AccYangle <= HORIZONTAL_DEGREE) {//if AccYangle is higher than HORIZONTAL_DEGREE
			digitalWrite(Relay_Ch1, HIGH);		//tilt motor moves down
			digitalWrite(Relay_Ch2, HIGH);		//tilt motor moves up
			digitalWrite(Relay_Ch3, HIGH);		//tilt motor moves up
			digitalWrite(Relay_Ch4, LOW);		//tilt motor moves up
			motor_status = 4;
		}

		printf("tilt motor initialization started\nMotor status: %d\n", motor_status);
		while (motor_status == 3 || motor_status == 4) {
			startInt = mymillis();
			//read ACC and GYR data
			readACC(accRaw);
			readGYR(gyrRaw);
			//convert Gyro raw to degrees per second
			rate_gyr_x = (float)gyrRaw[0] * G_GAIN;
			rate_gyr_y = (float)gyrRaw[1] * G_GAIN;
			rate_gyr_z = (float)gyrRaw[2] * G_GAIN;
			//Calculate the angles from the gyro
			gyroXangle += rate_gyr_x*DT;
			gyroYangle += rate_gyr_y*DT;
			gyroZangle += rate_gyr_z*DT;

			//Convert Accelerometer values to degrees
			AccXangle = (float)(atan2(accRaw[1], accRaw[2]) + M_PI)*RAD_TO_DEG;
			AccYangle = (float)(atan2(accRaw[2], accRaw[0]) + M_PI)*RAD_TO_DEG;

			AccXangle -= (float)180.0;
			if (AccYangle > 90)
				AccYangle -= (float)270;
			else
				AccYangle += (float)90;
			switch (motor_status) {
			case 3:
				if (AccYangle <= HORIZONTAL_DEGREE) {			//when AccYangle is higer than HORIZONTAL_DEGREE
					digitalWrite(Relay_Ch3, HIGH);
					printf("AccYangle %7.3f \n", AccYangle);			//motor stop
					printf("Tilt motor stopped\n");
					motor_status = 0;
				}
				break;
			case 4:
				if (AccYangle >= HORIZONTAL_DEGREE) {			//when AccYangle is lower than HORIZONTAL_DEGREE
					digitalWrite(Relay_Ch4, HIGH);			//motor stop
					printf("AccYangle %7.3f \n", AccYangle);			//motor stop
					printf("Tilt motor stopped\n");
					motor_status = 0;
				}
				break;
			}

			//
		}
		printf("Tilt motor initializing completed!\n");
		delay(20);
		//
		//pan motor part
		//
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		readMAG(magRaw);
		readACC(accRaw);
		float heading = 180 * atan2(magRaw[1], magRaw[0]) / M_PI;
		//convert heading to 0-360
		if (heading < 0)
			heading += 360;

		//	printf("head %7.3f \t ", heading);
		accXnorm = accRaw[0] / sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);
		accYnorm = accRaw[1] / sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);

		//Calculate pithc and roll
		pitch = asin(accXnorm);
		roll = -asin(accYnorm / cos(pitch));
		//Calculate the new tilt compensated values
		magXcomp = magRaw[0] * cos(pitch) + magRaw[2] * sin(pitch);
		if (LSM9DS0)
			magYcomp = magRaw[0] * sin(roll)*sin(pitch) + magRaw[1] * cos(roll) - magRaw[2] * sin(roll)*cos(pitch); // LSM9DS0
		else
			magYcomp = magRaw[0] * sin(roll)*sin(pitch) + magRaw[1] * cos(roll) + magRaw[2] * sin(roll)*cos(pitch); // LSM9DS1

																													//Calculate heading
		heading = 180 * atan2(magYcomp, magXcomp) / M_PI;
		//Convert heading to 0 - 360
		if (heading < 0)
			heading += 360;

		enqueueArray(heading);
		printf("Compensated Heading %7.3f %7.3f\n", heading, headingArray[0]);

		usleep(25000);
		if (calcAverage()>= SOUTH_DEGREE) {						//If heading value is bigger than 180, pen motor turns 
			digitalWrite(Relay_Ch1, LOW);					//pan motor starts turning counter clock wis
			digitalWrite(Relay_Ch2, HIGH);
			digitalWrite(Relay_Ch3, HIGH);
			digitalWrite(Relay_Ch4, HIGH);
			motor_status = 1;
		}
		else if (calcAverage() < SOUTH_DEGREE) {
			digitalWrite(Relay_Ch1, HIGH);
			digitalWrite(Relay_Ch2, LOW);
			digitalWrite(Relay_Ch3, HIGH);
			digitalWrite(Relay_Ch4, HIGH);
			motor_status = 2;
		}
		printf("motor_status: %d\n", motor_status);
		//move pan until heading North
		while (motor_status == 1 || motor_status == 2) {
			readMAG(magRaw);
			readACC(accRaw);
			float heading = 180 * atan2(magRaw[1], magRaw[0]) / M_PI;
			//convert heading to 0-360
			if (heading < 0)
				heading += 360;

			//	printf("head %7.3f \t ", heading);
			accXnorm = accRaw[0] / sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);
			accYnorm = accRaw[1] / sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);

			//Calculate pithc and roll
			pitch = asin(accXnorm);
			roll = -asin(accYnorm / cos(pitch));
			//Calculate the new tilt compensated values
			magXcomp = magRaw[0] * cos(pitch) + magRaw[2] * sin(pitch);
			if (LSM9DS0)
				magYcomp = magRaw[0] * sin(roll)*sin(pitch) + magRaw[1] * cos(roll) - magRaw[2] * sin(roll)*cos(pitch); // LSM9DS0
			else
				magYcomp = magRaw[0] * sin(roll)*sin(pitch) + magRaw[1] * cos(roll) + magRaw[2] * sin(roll)*cos(pitch); // LSM9DS1

																														//Calculate heading
			heading = 180 * atan2(magYcomp, magXcomp) / M_PI;
			//Convert heading to 0 - 360
			if (heading < 0)
				heading += 360;
			enqueueArray(heading);
			printf("%7.3f\t\t%7.3f\n", heading, calcAverage());
			switch (motor_status) {
			case 1:
				if (calcAverage() < SOUTH_DEGREE ) {				//when the motor becomes heading NORTH_DEGREE
					digitalWrite(Relay_Ch1, HIGH);				//motor stops
					printf("pan motor initialization is completed!\n");
					motor_status = 0;
				}
				break;
			case 2:
				if (calcAverage() >= SOUTH_DEGREE) {
					printf("pan motor initialization is completed!\n");
					digitalWrite(Relay_Ch2, HIGH);
					motor_status = 0;
				}
				break;
			}
			usleep(25000);
		}
		return 1;
	}
#endif
