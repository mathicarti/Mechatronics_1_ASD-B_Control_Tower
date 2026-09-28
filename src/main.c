#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "message.h"

#define INPUT_BUFFER_SIZE 1024

int main(void) 
{
	// Get input from terminal and 'clean it'
	char input_buffer[INPUT_BUFFER_SIZE];

	while (fgets(input_buffer, INPUT_BUFFER_SIZE, stdin) != NULL)
	{
		input_buffer[strlen(input_buffer) - 1] = '\0';

		// Check the type of data that was inputed (ADS-B or *time)
		if (input_buffer[0] == '*')
		{
			// Should just sort the *time into an structure and then just process it afterwards
			Time_Request current_request;
			int hours, minutes;

			char *token = strtok(input_buffer, ",");
			int i = 0;

			while (token != NULL)
			{
				if (i == 0)
				{
					sscanf(token, "*time:%d:%d,", &hours, &minutes);
					current_request.time = convert_to_time(hours, minutes);
				}
				else if (i == 1 && !strcmp(token, "close"))
					current_request.request_ID = CLOSING;
				
				else if (i == 1 && !strcmp(token, "est_pos"))
					current_request.request_ID = EST_POS;

				else if (i == 1 && !strcmp(token, "num_contacts"))
					current_request.request_ID = NUM_CONTACTS;

				else if (i == 1 && !strcmp(token, "check_separation"))
					current_request.request_ID = CHECK_SEPARATION;

				else if (i == 2 && current_request.request_ID != CLOSING && current_request.request_ID != NUM_CONTACTS)
					current_request.aircraft_id = atoi(token);

				else if (i == 3 && current_request.request_ID == CHECK_SEPARATION)
					current_request.minimum_sep_distance = (double) atof(token);

				i++;
				token = strtok(NULL, ",");
			}

			switch (current_request.request_ID)
			{
				case CLOSING:
					printf("closing\n");
					return 0;
				
				case EST_POS:
					printf("checking position of plane ID: %d\n", current_request.aircraft_id);
					break;
				
				case NUM_CONTACTS:
					printf("Checking number of contacts\n");
					break;

				case CHECK_SEPARATION:
					printf("Cheching separationof of plane ID: %d\n", current_request.aircraft_id);
					break;

				default:
					printf("Couldn't identify request type\n");
					return 4;
					break;
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
			test_packet.time = convert_to_time(hours, minutes);

			printf("ID: %d\nTime: %lf\nNorth: %lf\nEast: %lf\nAltitude: %d\nHead: %lf\nSpeed: %lf\n", test_packet.id, 
				test_packet.time, test_packet.north, test_packet.east, test_packet.altitude, test_packet.heading, 
				test_packet.speed);
		}
		else
		{
			printf("Invalid input\n");
			return 2;
		}
	}
	return 0;
}

double convert_to_time(int hours, int minutes)
{
	double total_hours = (double) hours + ( (double) minutes / 60.0 );
	// Round total_hours to 3 decimal places
	total_hours = round(total_hours * 1000.0) / 1000.0;
	return total_hours;
}
