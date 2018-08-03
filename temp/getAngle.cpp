#ifndef __GETANGLE__
#define __GETANGLE__ 
//#include <iostream>
//#include <cmath>
#include <math.h>


#define INITIAL_VELOCITY 120 //�ʱ�ӵ� 30m/s�� ����(�ӽ�)
#define PI 3.14159265358979323846
#define G 9.80665 //�߷°��ӵ�(m/s^2)
	
double deg2rad(double degree);
double rad2deg(double radian);

// lat ����   long �浵
// theta �Ҹ��� ȸ����   omega ƿƮ���� ȸ����

// �� ���� ������ �Ÿ� dis
// unit : m(statute miles), k(kilometers), n(nautical miles)
void getDistance(double lat1, double long1, double lat2, double long2, double* _dis);

void getTheta(double _dis, double* _theta);
void getOmega(double lat1, double long1, double lat2, double long2, double* _omega);

int calcAngleAndHeading(double* _loc_val, double* _theta, double* _omega) {


	double dis;// ��Ÿ�

	getDistance(_loc_val[0], _loc_val[1], _loc_val[2], _loc_val[3], &dis);
	//cout << "\nx:" << dis << endl;
	getTheta(dis, _theta);
	//cout << "theta:" << *_theta << endl; // ���� ��
	getOmega(_loc_val[2], _loc_val[3], _loc_val[0], _loc_val[1], _omega);
	//cout << "Omega:" << *_omega << endl; // ���ް� ��
	printf("angle: %7.5lf compass: %7.5lf\n", *_theta, *_omega);
	*_theta = rad2deg(*_theta);
	*_omega = rad2deg(*_omega);	
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
	// ���Ϲ������
	*_omega = atan2(lat2 - lat1, long2 - long1); // [-pi, +pi]
} 
#endif
