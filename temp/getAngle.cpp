#ifndef __GETANGLE__
#define __GETANGLE__ 
//#include <iostream>
//#include <cmath>
#include <math.h>


#define INITIAL_VELOCITY 120 //ÃÊ±â¼Óµµ 30m/s·Î ¼³Á¤(ÀÓ½Ã)
#define PI 3.14159265358979323846
#define G 9.80665 //Áß·Â°¡¼Óµµ(m/s^2)
	
double deg2rad(double degree);
double rad2deg(double radian);

// lat À§µµ   long °æµµ
// theta ÆÒ¸ðÅÍ È¸Àü°¢   omega Æ¿Æ®¸ðÅÍ È¸Àü°¢

// µÎ ÁöÁ¡ »çÀÌÀÇ °Å¸® dis
// unit : m(statute miles), k(kilometers), n(nautical miles)
void getDistance(double lat1, double long1, double lat2, double long2, double* _dis);

void getTheta(double _dis, double* _theta);
void getOmega(double lat1, double long1, double lat2, double long2, double* _omega);

int calcAngleAndHeading(double* _loc_val, double* _theta, double* _omega) {


	double dis;// »ç°Å¸®
	int i;
	for(i =0 ;i< 4; i++) {
		printf("%6.3lf\n", _loc_val[i]);
	}
	getDistance(_loc_val[1], _loc_val[0], _loc_val[3], _loc_val[2], &dis);
	//cout << "\nx:" << dis << endl;
	getTheta(dis, _theta);
	//cout << "theta:" << *_theta << endl; // ¶óµð¾È °ª
	getOmega(_loc_val[3], _loc_val[2], _loc_val[1], _loc_val[0], _omega);
	//cout << "Omega:" << *_omega << endl; // ¿À¸Þ°¡ °ª
	printf("angle: %7.5lf compass: %7.5lf\n", *_theta, *_omega);
	*_theta = rad2deg(*_theta);
	*_omega = rad2deg(*_omega);
	if(*_omega < 0)
		*_omega += 360;	
	//cin >> dis;
	printf("angle: %7.5lf compass: %7.5lf\n", *_theta, *_omega);

	return 0;
}


double deg2rad(double degree) {
	return degree * PI / 180;
}

double rad2deg(double radian) {
	return radian * 180 / PI;
}

void getDistance(double lat1, double long1, double lat2, double long2, double * _dis)
{
	double th;
	th = long1 - long2;
	*_dis = sin(deg2rad(lat1))*sin(deg2rad(lat2)) + cos(deg2rad(lat1))*cos(deg2rad(lat2))*cos(deg2rad(th));
	*_dis = acos(*_dis);
	*_dis = rad2deg(*_dis);
	*_dis = *_dis * 60 * 1.1515;
	*_dis = *_dis * 1.609344 * 1000; // conversion 'Meter'
}

void getTheta(double _dis, double*_theta)
{
	*_theta = 0.5*asin(G*(_dis) / pow(INITIAL_VELOCITY, 2)); // [-pi/2, +pi/2]
}

void getOmega(double lat1, double long1, double lat2, double long2, double * _omega)
{
	// Á¤ºÏ¹æÇâ±âÁØ
	//

	if ((long1-long2)<0 && (lat1-lat2)>0){
		printf("come on");
		*_omega = atan2(lat2 - lat1, long2 - long1) - 1.5*PI;
	}

	else{
		*_omega = atan2(lat2 - lat1, long2 - long1) + 1.5*PI;
	}	
	
	printf("omega : %7.5lf", *_omega);
} 
#endif
