#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "message.h"

#define INPUT_BUFFER_SIZE 1024

int main(void) 
{
	// Get input from terminal and 'clean it'
	char input_buffer[INPUT_BUFFER_SIZE];

	if (fgets(input_buffer, INPUT_BUFFER_SIZE, stdin) == NULL)
	{
		printf("Failed to get input...\n");
		return 1;
	}
	input_buffer[strlen(input_buffer) - 1] = '\0';

	// Check the type of data that was inputed (ADS-B or *time...)
	if (input_buffer[0] == '*')
	{
		char *token = strtok(input_buffer, ",");
		int i = 0;

		while (token != NULL)
		{
			if (i == 1 && !strcmp(token, "close"))
			{
				printf("closing\n");
				return 0;
			}

			i++;
			token = strtok(NULL, ",");
		}
	}
	else if (input_buffer[0] == '#')
	{
		ADSBPacket test_packet;
		int hours, minutes;

		if (sscanf(input_buffer, "#ADS-B:%d,time:%d:%d,N:%lf,E:%lf,alt:%d,head:%lf,speed:%lf\n", &(test_packet.id), &hours,
			&minutes, &(test_packet.north), &(test_packet.east), &(test_packet.altitude), &(test_packet.heading), 
			&(test_packet.speed)) != 8)
		{
			printf("Failed to correctly parse\n");
			return 3;
		}
		printf("ID: %d\nTime: %d:%d\nNorth: %lf\nEast: %lf\nAltitude: %d\nHead: %lf\nSpeed: %lf\n", test_packet.id, 
			hours, minutes, test_packet.north, test_packet.east, test_packet.altitude, test_packet.heading, 
			test_packet.speed);
	}
	else
	{
		printf("Invalid input\n");
		return 2;
	}


	return 0;
}
