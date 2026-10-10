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
					request->minimum_sep_distance = to_m((double) atof(token));
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
	double north_pos, east_pos;
	
	if (packet_node)
		estimate_position(request.time, packet_node->packet, &north_pos, &east_pos);
	
	if (!packet_node || !in_airspace(north_pos, east_pos))
	{
		printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
		return;
	}
	double dt_issue;

	if (!find_first_separation_issue(packet_node_head, packet_node->packet, (double) request.time, request.minimum_sep_distance, &dt_issue))
	{
		printf("No separation issues\n");
		return;
	}
	estimate_position(request.time + dt_issue, packet_node->packet, &north_pos, &east_pos);

	if (in_airspace(north_pos, east_pos))
		printf("Separation issue: N:%.1lf,E:%.1lf\n", to_km(north_pos), to_km(east_pos));
	
	else
		printf("No separation issues\n");
}

int find_first_separation_issue(Packet_Node *packet_node_head, ADSBPacket packet, double t_check, double minimum_sep_distance, double *dt_issue)
{
	int found = 0;
	double min_dt_issue = 0;

	for (Packet_Node *cursor = packet_node_head; cursor != NULL; cursor = cursor->next)
	{
		if (cursor->packet.id == packet.id)
			continue;

		double north_pos, east_pos;
		
		estimate_position(t_check, cursor->packet, &north_pos, &east_pos);
		if (!in_airspace(north_pos, east_pos))
			continue;

		double dt_issue_pair;

		if (check_pair_for_issue(t_check, packet, cursor->packet, minimum_sep_distance, &dt_issue_pair) 
			&& (!found || dt_issue_pair < min_dt_issue))
		{
			min_dt_issue = dt_issue_pair;
			found = 1;
		}
	}
	if (found)
		*dt_issue = min_dt_issue;
	return found;
}

int check_pair_for_issue(double t_check, ADSBPacket a, ADSBPacket b, double minimum_sep_distance, double *dt_issue)
{
	// Altitude Check
	if (abs(a.altitude - b.altitude) > MAX_ALT_DIFF_M)
		return 0;

	double a_north_pos, a_east_pos, a_north_vel, a_east_vel;
	double b_north_pos, b_east_pos, b_north_vel, b_east_vel;

	estimate_position(t_check, a, &a_north_pos, &a_east_pos);
	estimate_position(t_check, b, &b_north_pos, &b_east_pos);

	compute_velocity(a, &a_north_vel, &a_east_vel);
	compute_velocity(b, &b_north_vel, &b_east_vel);

	double dt_north_pos = b_north_pos - a_north_pos;
	double dt_east_pos = b_east_pos - a_east_pos;
	double dt_north_vel = b_north_vel - a_north_vel;
	double dt_east_vel = b_east_vel - a_east_vel;

	double quad_a = (dt_north_vel * dt_north_vel) + (dt_east_vel * dt_east_vel);
	double quad_b = 2 * ((dt_north_vel * dt_north_pos) + (dt_east_vel * dt_east_pos));
	double quad_c = (dt_north_pos * dt_north_pos) + (dt_east_pos * dt_east_pos) - (minimum_sep_distance * minimum_sep_distance);
	
	double root_1, root_2;
	int real_roots = solve_quadratic(quad_a, quad_b, quad_c, &root_1, &root_2);

	// Never in range, or time in the past
	if (real_roots == 0 || root_2 < 0)
		return 0;

	// root_1 < 0 <= root_2 already in range
	*dt_issue = (root_1 < 0) ? 0 : root_1;
	return 1;
}

void estimate_position(double t_check, ADSBPacket packet, double *north_pos, double *east_pos)
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
