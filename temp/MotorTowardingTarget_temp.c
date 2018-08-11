#ifndef MOTOR_TOWARDING_TARGET
#define MOTOR_TOWARDING_TARGET

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
#include <stdbool.h>
#include "getAngle.cpp"

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
#define clear() printf("\033[H\033[J")
double ttArray[10];
double tpArray[10];

double TcalcAverage(double* Arr) {
	int i;
	int num = 0;
	double sum = 0;
	double min = DBL_MAX, max = -DBL_MAX;
	for (i = 0; i< 10; i++) {
		if (Arr[i] < min)
			min = Arr[i];
		if (Arr[i] > max)
			max = Arr[i];
	}

	for (i = 0; i< 10; i++) {
		if (Arr[i] != -DBL_MAX) {
			if (max - min <= 270) {
				num++;
				sum += Arr[i];
			}
			else {
				num++;
				if (Arr[i] > 270) {
					sum += Arr[i] - 360;
				}
				else {
					sum += Arr[i];
				}
			}
		}
	}
	if (sum < 0)
		return (sum / num + 360);
	else
		return (sum / num);
}

void TenqueueArray(double* arr, double val) {
	int idx = 0;
	for (idx = 0; idx< 10; idx++) {
		if (arr[idx] == -DBL_MAX) {
			arr[idx] = val;
			return;
		}
	}
	for (idx = 0; idx<9; idx++) {
		arr[idx] = arr[idx + 1];
	}
	arr[9] = val;
	return;
}

int readAngle(int* _accRaw, int* _gyrRaw, double* _AccXangle, double* _AccYangle) {
	float rate_gyr_x = 0.0;		// [deg/sec]
	float rate_gyr_y = 0.0;		// [deg/sec]
	float rate_gyr_z = 0.0;		// [deg/sec]

	float gyroXangle = 0.0;
	float gyroYangle = 0.0;
	float gyroZangle = 0.0;
	float CFangleX = 0.0;
	float CFangleY = 0.0;

	readACC(_accRaw);		//read values from accelerometer to accRaw array
	readGYR(_gyrRaw);		//read values from gyroscope to gryRaw array

	//convert Gyro raw to degrees per second
	rate_gyr_x = (double)_gyrRaw[0] * G_GAIN;
	rate_gyr_y = (double)_gyrRaw[1] * G_GAIN;
	rate_gyr_z = (double)_gyrRaw[2] * G_GAIN;
	//Calculate the angles from the gyro
	gyroXangle += rate_gyr_x*DT;
	gyroYangle += rate_gyr_y*DT;
	gyroZangle += rate_gyr_z*DT;
	//Convert Accelerometer values to degrees
	*_AccXangle = (double)(atan2(_accRaw[1], _accRaw[2]) + M_PI)*RAD_TO_DEG;
	*_AccYangle = (double)(atan2(_accRaw[2], _accRaw[0]) + M_PI)*RAD_TO_DEG;
	//print angle values
	*_AccXangle -= (double)180.0;
	if (*_AccYangle > 90)
		*_AccYangle -= (double)270;
	else
		*_AccYangle += (double)90;
	*_AccYangle *=-1;
	return 1;
}

int readCompass(int *_magRaw, int *_accRaw, double* _heading) {
	//variables that is needed for compass
	float accXnorm, accYnorm, pitch, roll, magXcomp, magYcomp;

	readMAG(_magRaw);
	readACC(_accRaw);

	accXnorm = _accRaw[0] / sqrt(_accRaw[0] * _accRaw[0] + _accRaw[1] * _accRaw[1] + _accRaw[2] * _accRaw[2]);
	accYnorm = _accRaw[1] / sqrt(_accRaw[0] * _accRaw[0] + _accRaw[1] * _accRaw[1] + _accRaw[2] * _accRaw[2]);
	//Calculate pithc and roll
	pitch = asin(accXnorm);
	roll = -asin(accYnorm / cos(pitch));
	//Calculate new tilt compensated values
	magXcomp = _magRaw[0] * cos(pitch) + _magRaw[2] * sin(pitch);
	if (LSM9DS0)
		magYcomp = _magRaw[0] * sin(roll)*sin(pitch) + _magRaw[1] * cos(roll) - _magRaw[2] * sin(roll)*cos(pitch); // LSM9DS0
	else
		magYcomp = _magRaw[0] * sin(roll)*sin(pitch) + _magRaw[1] * cos(roll) + _magRaw[2] * sin(roll)*cos(pitch); // LSM9DS1

																												//Calculate heading
	*_heading = (double)(180 * atan2(magYcomp, magXcomp) / M_PI);

	if (*_heading < 0)
		*_heading += 360;

	printf("Compensated Heading %7.3f \n", *_heading);
}

int moveTiltToTarget(double* ttArr, double towardAngle, int* _accRaw, int* _gyrRaw, double* _AccXangle, double* _AccYangle, bool* flag) {
	printf("Average Angle: %6.3lf\n, target Angle: %6.3lf\n", TcalcAverage(ttArr), towardAngle);
	if (TcalcAverage(ttArr) < towardAngle )
		*flag = true;
	else if(abs(TcalcAverage(ttArr) - towardAngle) < 180)
		*flag = false;
	else
		*flag = true;
	if (*flag == true) {										//if now angle is lower than toward angle
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, LOW);
		digitalWrite(Relay_Ch4, HIGH);
		while (TcalcAverage(ttArr) < towardAngle) {
			readAngle(_accRaw, _gyrRaw, _AccXangle, _AccYangle);
			TenqueueArray(ttArr, *_AccYangle);
			usleep(25000);
		}
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		if (abs(TcalcAverage(ttArr) - towardAngle) < 1.0) {
			printf("Tilt motor towarding correctly!! (now angle: %7.4lf towardAngle: %7.4lf\n", TcalcAverage(ttArr), towardAngle);
			return 1;
		}
		else {
			moveTiltToTarget(ttArr, towardAngle, _accRaw, _gyrRaw, _AccXangle, _AccYangle, flag);
		}
	}
	else {
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, LOW);
		while (TcalcAverage(ttArr) > towardAngle) {
			readAngle(_accRaw, _gyrRaw, _AccXangle, _AccYangle);
			TenqueueArray(ttArr, *_AccYangle);
			usleep(25000);
		}
		if (abs(TcalcAverage(ttArr) - towardAngle) < 1.0) {
			printf("Tilt motor towarding correctly!! (now angle: %7.4lf towardAngle: %7.4lf)\n", TcalcAverage(ttArr), towardAngle);
			return 1;
		}
		else {
			moveTiltToTarget(ttArr, towardAngle, _accRaw, _gyrRaw, _AccXangle, _AccYangle, flag);
		}
	}
}
int movePanToTarget(double* tpArr, double towardCompass, int* _magRaw, int* _accRaw, double* _heading, bool* flag, int* _motorstatus) {
	if (_motorstatus == 0){
		if(towardCompass<180) {
			*flag = true;
			*_motorstatus = 1;
		}
		else if(towardCompass >=180) {
			*flag = false;
			*_motorstatus =2;
		}
	}
	else {
		if(TcalcAverage(tpArr) < towardCompass) {
			*flag = true;
		}
		else if (TcalcAverage(tpArr) >= towardCompass) {
			*flag = false;
		}
	}
	int i;
	if(*flag == true){
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, LOW);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		while(TcalcAverage(tpArr) < towardCompass) {
			readCompass(_magRaw, _accRaw, _heading);
			TenqueueArray(tpArr, *_heading);
			//clear();
			printf("now compansated heading: %7.4lf\ttarget degree: %7.4lf\n", TcalcAverage(tpArr), towardCompass);
			usleep(25000);
		}
		//clear();
		//printf("now compansated heading: %7.4lf\ttarget degree: %7.4lf\n", TcalcAverage(tpArr), towardCompass);
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		if(abs(TcalcAverage(tpArr) - towardCompass) <2.0) {
			for (i =0; i< 10; i++){
				printf("%3.1lf \t", tpArr[i]);
			}
			printf("\n");
			printf("Pan motor towarding correctly!!! (now angle: %7.4lf target angle: %7.4lf)", TcalcAverage(tpArr), towardCompass);
			*_motorstatus =0;
			return 1;
		}
		else {
		//:	movePanToTarget(tpArr, towardCompass, _magRaw, _accRaw, _heading, flag, _motorstatus);
		}
	}
	
	else if(*flag == false) {
		digitalWrite(Relay_Ch1, LOW);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		while(TcalcAverage(tpArr) > towardCompass) {
			readCompass(_magRaw, _accRaw, _heading);
			TenqueueArray(tpArr, *_heading);
			//clear();
			printf("now compansated heading: %7.4lf\ttarget degree: %7.4lf\n", TcalcAverage(tpArr), towardCompass);
			usleep(25000);
		}
		//clear();
		//printf("now compansated heading: %7.4lf\ttarget degree: %7.4lf\n", TcalcAverage(tpArr), towardCompass);
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		if(abs(TcalcAverage(tpArr) - towardCompass) < 2.0) {
			
			for (i =0; i< 10; i++){
				printf("%3.1lf \t", tpArr[i]);
			}
			printf("\n");
			printf("Pan motor towarding correctly!!! (now angle: %7.4lf target angle: %7.4lf)", TcalcAverage(tpArr), towardCompass);
			*_motorstatus =0;
			return 1;
		}
		else {
		//	movePanToTarget(tpArr, towardCompass, _magRaw, _accRaw, _heading, flag, _motorstatus);
		}
	}
}
			
int motorMoveToTarget(double* _loc_val)
{
	int i;
	for (i = 0; i < 10; i++) {
		ttArray[i] = -DBL_MAX;
		tpArray[i] = -DBL_MAX;
	}
	//_loc_val is a 4 size array; [0]: target latitude [1]: target longitude [2]: mortar latitude [3]: mortar longitude
	int accRaw[3];				//raw value of accelerometer (x, y, z)
	int gyrRaw[3]; 				//raw value of gyroscope (x, y, z)
	int magRaw[3];				//raw value of magnetometer (x, y, z)

	double AccYangle = 0.0;
	double AccXangle = 0.0;

	int startInt = mymillis();
	struct timeval tvBegin, tvEnd, tvDiff;

	int motor_status = 0; 	//0: stop, 1: right, 2: left, 3: up, 4: down
	//variables that save toward targetting angle & direction
	double towardAngle, towardCompass;
	double heading;
	bool flag = true;					//if flag is true, flag motor goes up/counter clock wise
										//else motor goes down, conter clock wise
	calcAngleAndHeading(_loc_val, &towardAngle, &towardCompass);
	if(towardCompass <0)
		towardCompass +=360;
	printf("%8.5lf %8.5lf", towardAngle, towardCompass);
	signal(SIGINT, INThandler);

	detectIMU();				//IMU detect
	enableIMU();				//enabling IMU

	gettimeofday(&tvBegin, NULL);	//assign timevalue to tvBegin
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	readAngle(accRaw, gyrRaw, &AccXangle, &AccYangle);
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
	//readAngle(accRaw, gyrRaw, &AccXangle, &AccYangle);
	//tilt until being horizon
	if (towardAngle > 0) {		//if AccYangle is lower than HORIZONTAL_DEGREE
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, HIGH);
		digitalWrite(Relay_Ch3, LOW);
		digitalWrite(Relay_Ch4, HIGH);
		motor_status = 3;
	}
	else if (towardAngle <0) {//if AccYangle is higher than HORIZONTAL_DEGREE
		printf("Mortar cannot heading lower than 0 degree!!");
	}

	printf("tilt motor initialization started\nMotor status: %d\n", motor_status);

	//
	//pan motor part
	//
	motor_status =0;
	readCompass(magRaw, accRaw, &heading);
	TenqueueArray(tpArray, heading);
	printf("Compensated Heading %7.3f \n", heading);
	movePanToTarget(tpArray, towardCompass, magRaw, accRaw, &heading, &flag, &motor_status);
	delay(3000);	
	TenqueueArray(ttArray, AccYangle);
	moveTiltToTarget(ttArray, towardAngle,  accRaw, gyrRaw, &AccXangle, &AccYangle, &flag);
	usleep(25000);

/*	if (towardCompass < 180) {						//If heading value is bigger than 180, pen motor turns 
		digitalWrite(Relay_Ch1, LOW);					//pan motor starts turning counter clock wise
//		digitalWrite(Relay_Ch2, HIGH);
//		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		motor_status = 1;
	}
	else if (towardCompass >= 180) {
		digitalWrite(Relay_Ch1, HIGH);
		digitalWrite(Relay_Ch2, LOW);
		digitalWrite(Relay_Ch3, HIGH);
		digitalWrite(Relay_Ch4, HIGH);
		motor_status = 2;
	}
	printf("motor_status: %d\n", motor_status);
	
	//move pan until heading North 
	////////////////////////////////////////////////////////////////
	while (motor_status == 1 || motor_status == 2) {
		readCompass(magRaw, accRaw, &heading);
		printf("%7.3f\n", heading);
		switch (motor_status) {
		case 1:
			if (heading > towardCompass && heading <=180) {				//when the motor becomes heading NORTH_DEGREE
				digitalWrite(Relay_Ch1, HIGH);			//motor stops
				printf("pan motor setting is completed!\n");
				motor_status = 0;
			}
			break;
		case 2:
			if (heading <= towardCompass && heading >= 180) {
				printf("pan motor setting is completed!\n");
				digitalWrite(Relay_Ch2, HIGH);
				motor_status = 0;
			}
			break;
		}
		usleep(25000);
	}*/

	return 0;
}

#endif

