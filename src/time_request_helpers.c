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
	int i = 0;

	while (token != NULL)
	{
		if (i == 0)
		{
			sscanf(token, "*time:%d:%d,", &hours, &minutes);
			request->time = convert_to_time(hours, minutes);
		}
		else if (i == 1)
		{
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
		}
		else if (i == 2 && request->request_ID != CLOSING && request->request_ID != NUM_CONTACTS)
			request->id = atoi(token);

		else if (i == 3 && request->request_ID == CHECK_SEPARATION)
			request->minimum_sep_distance = (double) atof(token);

		i++;
		token = strtok(NULL, ",");
	}
}

int handle_est_pos(Time_Request request, Packet_Node *packet_node_head)
{
    double est_pos_n, est_pos_e;
    int ret = get_est_pos(request, packet_node_head, &est_pos_n, &est_pos_e);

    if (ret != 0)
    {
        printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
        return 1;
    }

    printf("Aircraft (ID:%i): Estimated Position: N:%.1lf,E:%.1lf\n", request.id, to_km(est_pos_n), to_km(est_pos_e));
    return 2;
}

int get_est_pos(Time_Request request, Packet_Node *packet_node_head, double *est_pos_n, double *est_pos_e)
{
    Packet_Node *packet_node;
    
	// Check if id exists
	if ((packet_node = get_packet_node(packet_node_head, request.id)) == NULL)
        return 1;

	double pn_t = get_current_north_pos(request, packet_node);
	double pe_t = get_current_east_pos(request, packet_node);
		
	// Check if pn_t and pe_t is in airspace
	if(!in_airspace(pn_t, pe_t))
		return 2;

	// Return pass and set pointers to pos
	*est_pos_n = pn_t;
	*est_pos_e = pe_t;

	return 0;
}

int handle_num_contacts(Time_Request request, Packet_Node *packet_node_head)
{
    printf("Currently tracking %i aircraft\n", get_num_contacts(request, packet_node_head));
	return 1;
}

int get_num_contacts(Time_Request request, Packet_Node *packet_node_head)
{
	unsigned int tracking = 0;
	
	while (packet_node_head)
	{
		// Checks if the updated positions of the IDs are in the airspace
		if (in_airspace(get_current_north_pos(request, packet_node_head), get_current_east_pos(request, packet_node_head)))
			tracking++;
		
		packet_node_head = packet_node_head->next;
	}
	return tracking;
}

double get_current_north_pos(Time_Request request, Packet_Node *packet_node)
{
	double v_n = packet_node->packet.speed * sin(to_radians(packet_node->packet.heading));
	double pn_t = packet_node->packet.north + v_n * (request.time - packet_node->packet.time);

	return pn_t;
}

double get_current_east_pos(Time_Request request, Packet_Node *packet_node)
{
	double v_e = packet_node->packet.speed * cos(to_radians(packet_node->packet.heading));
	double pe_t = packet_node->packet.east + v_e * (request.time - packet_node->packet.time);

	return pe_t;
}

// Checks if point (x, y) -> (pn, pe) is within a 350km radius, return 0 if in airspace, 1 if not
int in_airspace(double pn, double pe)
{
	return ((pn * pn) + (pe * pe)) <= (AIRSPACE_RADIUS * AIRSPACE_RADIUS);
}
