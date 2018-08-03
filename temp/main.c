#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <sys/socket.h>
//#include <bluetooth/bluetooth.h>
//#include <bluetooth/sdp.h>
//#include <bluetooth/sdp_lib.h>
//#include <bluetooth/rfcomm.h>
#include "MotorInitializing.c"
#include "BluetoothServer.c"
#include "MotorTowardingTarget.c"
#include "gprmc.c"
int main()
{
	double loc_val[4] = { 0, 0, 0, 0 };
	if (motorInitializing() == 1) {
		printf("[MOTOR INITIALIZING IS FINISHED!!!]\n");
		printf("[BLUETOOTH SERVER OPEN!!!]\n");
	}
	getMortarLocation(loc_val);
	if (startBluetoothServer(loc_val) == 1) {
		printf("[BLUETOOTH SEVER CLOSED!!!]\n");
	}
	else {
		printf("[BLUETOOTH CONNECTION FAILED!!!]\n");
		printf("[PROCESS IS TERMINATED!!!]\n");
		return 0;
	}
	getMortarLocation(loc_val);
	printf("[MORTAR LOCATION DATA IS COLLECTED!!!\n");
	printf("[1. TARGET LOCATION]\nLATITUDE: %9.6lf, LONGITUDE: %9.6lf\n[2. MORTAR LOCATION]\nLATITUDE: %9.6lf, LONGITUDE: %9.6lf\n", loc_val[0], loc_val[1], loc_val[2], loc_val[3]);
	motorMoveToTarget(loc_val);
	printf("[TARGETTING IS FINISHED!!!]");

	return 0;
}
