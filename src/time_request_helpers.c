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
            handle_check_separation(request, packet_node_head);
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

	estimate_position(request.time, packet_node->packet, north_pos, east_pos);
	
	if(!in_airspace(*north_pos, *east_pos))
		return 2;

	return 0;
}

void handle_num_contacts(Time_Request request, Packet_Node *packet_node_head)
{
    printf("Currently tracking %i aircraft\n", get_num_contacts(request.time, packet_node_head));
}

int get_num_contacts(int t_check, Packet_Node *packet_node_head)
{
	unsigned int tracking = 0;

	for (Packet_Node *cursor = packet_node_head; cursor != NULL; cursor = cursor->next)
	{
		double north_pos, east_pos;
		estimate_position(t_check, cursor->packet, &north_pos, &east_pos);

		if (in_airspace(north_pos, east_pos))
			tracking++;
	}
	return tracking;
}

void handle_check_separation(Time_Request request, Packet_Node *packet_node_head)
{
	Packet_Node *packet_node = get_packet_node(packet_node_head, request.id);
	
	if (packet_node)
	{
		printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
		return;
	}

	double north_pos, east_pos;
	estimate_position(request.time, packet_node->packet, &north_pos, &east_pos);

	if (in_airspace(north_pos, east_pos))
	{
		printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
		return;
	}

	double dt_issue;
	int res = find_first_separation_issue(packet_node_head, packet_node->packet, request.time, request.minimum_sep_distance, &dt_issue);

	if (!res)
	{
		printf("No separation issues\n");
	}

	estimate_position((request.time + dt_issue), packet_node->packet, &north_pos, &east_pos);

	printf("Separation issue: N:%i,E:%i", north_pos, east_pos);
}

int find_first_separation_issue(Packet_Node *packet_node_head, ADSBPacket packet, int t_check, double minimum_sep_distance, double *dt_issue)
{
	double min_dt_issue = -1;
	int min_dt_id;

	while (packet_node_head)
	{
		if (packet_node_head->packet.id == packet.id) 
		{
			packet_node_head = packet_node_head->next;
			continue;
		}
		int res = check_pair_for_issue(t_check, packet, packet_node_head->packet, minimum_sep_distance, dt_issue);

		if (res = 0)
			continue;

		if (min_dt_issue == -1) 
		{
			min_dt_issue = *dt_issue;
			min_dt_id = packet_node_head->packet.id;
		}
		else if (*dt_issue < min_dt_issue)
		{
			min_dt_issue = *dt_issue;
			min_dt_id = packet_node_head->packet.id;
		}
		packet_node_head = packet_node_head->next;
	}
	if (min_dt_issue == -1)
	{
		return 0;
	}

	min_dt_issue = *dt_issue;
	return 1;
}

int check_pair_for_issue(int t_check, ADSBPacket a, ADSBPacket b, double minimum_sep_distance, double *dt_issue)
{
	// Altitude Check
	if (abs(a.altitude - b.altitude) > minimum_sep_distance)
		return 0;

	double a_north_pos, a_east_pos, a_north_vel, a_east_vel;
	double b_north_pos, b_east_pos, b_north_vel, b_east_vel;

	estimate_position(t_check, a, &a_north_pos, &a_east_pos);
	estimate_position(t_check, b, &b_north_pos, &b_east_pos);

	compute_velocity(a, &a_north_vel, &a_east_vel);
	compute_velocity(b, &b_north_vel, &b_east_vel);

	double dt_north_pos = b_north_pos - a_north_pos;
	double dt_east_pos = b_east_pos - b_east_pos;
	double dt_north_vel = b_north_vel - a_north_vel;
	double dt_east_vel = b_east_vel - a_east_vel;

	double root_1, root_2;
	double quad_a = (dt_north_vel * dt_north_vel) + (dt_east_vel * dt_east_vel);
	double quad_b = 2 * ((dt_north_vel * dt_north_pos) + (dt_east_vel * dt_east_pos));
	double quad_c = (dt_north_pos * dt_north_pos) + (dt_east_pos * dt_east_pos) - (minimum_sep_distance * minimum_sep_distance);

	int real_roots = solve_quadratic(quad_a, quad_b, quad_c, &root_1, &root_2);

	switch (real_roots)
	{
		case 0:
			return 0;

		case 1:
			// In range in the past
			if (root_1 < 0)
				// Never in range
				return 0;
			// Upcoming collision
			else
			{
				*dt_issue = t_check - root_1;
				return 1;
			}
		case 2:
			// Currently within range
			if ((root_1 > 0 && root_2 < 0) || (root_1 < 0 && root_2 > 0))
			{
				*dt_issue = root_1 > root_2 ? t_check - root_1 : t_check - root_2;
				return 2;
			}
			// Upcoming collision
			else if (root_1 > 0 && root_2 > 0)
			{
				*dt_issue = root_1 < root_2 ? t_check - root_1 : t_check - root_2;
				return 3;
			}
	}
}

void estimate_position(int t_check, ADSBPacket packet, double *north_pos, double *east_pos)
{
	const double dt_seconds = t_check - packet.time;
	double north_vel, east_vel;

	compute_velocity(packet, &north_vel, &east_vel);
	
	*north_pos = packet.north + north_vel * dt_seconds;
	*east_pos = packet.east + east_vel * dt_seconds;
}

void compute_velocity(ADSBPacket packet, double *north_vel, double *east_vel)
{
	const double heading_rad = to_radians(packet.heading);

	*north_vel = packet.speed * sin(heading_rad);
	*east_vel = packet.speed * cos(heading_rad);
}

int in_airspace(double north_pos, double east_pos)
{
	return ((north_pos * north_pos) + (east_pos * east_pos)) <= (AIRSPACE_RADIUS * AIRSPACE_RADIUS);
}
