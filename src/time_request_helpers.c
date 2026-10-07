#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "message.h"

int handle_time_request(char *input, Packet_Node *packet_node_head)
{
    Time_Request request = {0};
    parse_time_request(&request, input);

    switch (request.request_ID)
    {
        case CLOSING:
            printf("closing\n");
            return 1;
        
        case EST_POS:
            handle_est_pos(request, packet_node_head);
            break;
        
        case NUM_CONTACTS:
            handle_num_contacts(request, packet_node_head);
            break;

        case CHECK_SEPARATION:
            printf("Not done\n");
            break;

        case UNDEFINED:
            printf("Couldn't identify request type\n");
            return 2;
    }

    return 0;
}

void parse_time_request(Time_Request *request, char *input)
{
	int hours, minutes;

	char *token = strtok(input, ",");
	int field_index = 0;

	while (token != NULL)
	{
		switch (field_index)
		{
			case 0:
				sscanf(token, "*time:%d:%d,", &hours, &minutes);
				request->time = convert_to_time(hours, minutes);
				break;

			case 1:
				if (!strcmp(token, "close"))
					request->request_ID = CLOSING;
				else if (!strcmp(token, "est_pos"))
					request->request_ID = EST_POS;
				else if (!strcmp(token, "num_contacts"))
					request->request_ID = NUM_CONTACTS;
				else if (!strcmp(token, "check_separation"))
					request->request_ID = CHECK_SEPARATION;
				else
					request->request_ID = UNDEFINED;
				break;
			
			case 2:
				if (request->request_ID != CLOSING && request->request_ID != NUM_CONTACTS)
					request->id = atoi(token);				
				break;

			case 3:
				if (request->request_ID == CHECK_SEPARATION)
					request->minimum_sep_distance = (double) atof(token);
				break;
		}
		field_index++;
		token = strtok(NULL, ",");
	}
}

void handle_est_pos(Time_Request request, Packet_Node *packet_node_head)
{
    double north_pos, east_pos;
    int ret = get_est_pos(request, packet_node_head, &north_pos, &east_pos);

    if (ret != 0)
    {
        printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
        return;
    }

    printf("Aircraft (ID:%i): Estimated Position: N:%.1lf,E:%.1lf\n", request.id, to_km(north_pos), to_km(east_pos));
    return;
}

int get_est_pos(Time_Request request, Packet_Node *packet_node_head, double *north_pos, double *east_pos)
{
    Packet_Node *packet_node = get_packet_node(packet_node_head, request.id);
    
	// Check if id exists
	if (packet_node == NULL)
        return 1;

	estimate_position(request, packet_node, north_pos, east_pos);
	
	if(!in_airspace(*north_pos, *east_pos))
		return 2;

	return 0;
}

void handle_num_contacts(Time_Request request, Packet_Node *packet_node_head)
{
    printf("Currently tracking %i aircraft\n", get_num_contacts(request, packet_node_head));
}

int get_num_contacts(Time_Request request, Packet_Node *packet_node_head)
{
	unsigned int tracking = 0;

	for (Packet_Node *cursor = packet_node_head; cursor != NULL; cursor = cursor->next)
	{
		double north_pos, east_pos;
		estimate_position(request, cursor, &north_pos, &east_pos);

		if (in_airspace(north_pos, east_pos))
			tracking++;
	}
	return tracking;
}

void estimate_position(Time_Request request, Packet_Node *packet_node, double *north_pos, double *east_pos)
{
	const double dt_seconds = request.time - packet_node->packet.time;
	const double heading_rad = to_radians(packet_node->packet.heading);

	double north_vel = packet_node->packet.speed * sin(heading_rad);
	double east_vel = packet_node->packet.speed * cos(heading_rad);
	
	*north_pos = packet_node->packet.north + north_vel * dt_seconds;
	*east_pos = packet_node->packet.east + east_vel * dt_seconds;
}

int in_airspace(double north_pos, double east_pos)
{
	return ((north_pos * north_pos) + (east_pos * east_pos)) <= (AIRSPACE_RADIUS * AIRSPACE_RADIUS);
}
