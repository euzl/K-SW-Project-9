#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(void)
{
	FILE *fp = fopen("g_result.txt", "r");

	char buf[256];
	char *array[20];
	while(fgets(buf, sizeof(buf), fp))
	{
		if(strstr(buf, "$GPGLL"))
		{
			int count = 0;
			char *token;
			token = strtok(buf, ",");
			while(token != NULL)
			{
				array[count++]=token;
				token = strtok(NULL, ",");
				if(count==20)
					break;
			}

			double tmp=0;
			double lat = atof(array[1]);
			tmp = (double)((int)lat/100);
			lat = lat/100;
			lat = lat-tmp;
			lat = lat*100.0;
			lat = lat/60.0;
			lat = lat + tmp;
				

			if(!strcmp(array[2],"S"))
				lat = -lat;
			
			double lon = atof(array[3]);
			
			tmp = (double)((int)lon/100);
			lon = lon/100;
			lon = lon-tmp;
			lon = lon*100.0;
			lon = lon/60.0;
			lon = lon + tmp;
			
			if(!strcmp(array[4],"W"))
				lon = -lon;

			printf("Latitude : %9.6lf\n",lat);
			printf("Longtitude : %9.6lf\n", lon);
		}
	}
	return 0;
}
