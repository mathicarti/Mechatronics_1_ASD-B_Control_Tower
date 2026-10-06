#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "message.h"

int main(void) 
{
	char input_buffer[INPUT_BUFFER_SIZE];
	
	Packet_Node *packet_node_head = NULL;
	
	// Get input from terminal and 'clean it'
	while (fgets(input_buffer, INPUT_BUFFER_SIZE, stdin) != NULL)
	{
		input_buffer[strlen(input_buffer) - 1] = '\0';
		
		// Check the type of data that was inputted (ADS-B or *time)
		if (input_buffer[0] == '*')
		{
			// Should just sort the *time into an structure and then just process it afterwards
			Time_Request request = {0};
			parse_time_request(&request, input_buffer);

			Packet_Node *request_node_buffer = get_packet_node(packet_node_head, request.id);

			switch (request.request_ID)
			{
				case CLOSING:
					printf("closing\n");
					free_packet_node(packet_node_head);
					return 0;
				
				case EST_POS:
					double est_pos_n, est_pos_e;

					int ret = get_est_position(packet_node_head, request, &est_pos_n, &est_pos_e);

					if (ret != 0)
					{
						printf("Aircraft (ID:%i) not currently in area of operation\n", request.id);
						break;
					}

					printf("Aircraft (ID:%i): Estimated Position: N:%.1lf,E:%.1lf\n", request.id, est_pos_n, est_pos_e);
					break;
				
				case NUM_CONTACTS:
					if (!request_node_buffer) printf("Couldn't find ID: %d in list\n", request.id);
					printf("Found ID: %d with time (minutes): %d\n", request_node_buffer->packet.id, request_node_buffer->packet.time);
					
					break;

				case CHECK_SEPARATION:
					if (!request_node_buffer) printf("Couldn't find ID: %d in list\n", request.id);
					printf("Found ID: %d with time (minutes): %d\n", request_node_buffer->packet.id, request_node_buffer->packet.time);
					
					break;

				case UNDEFINED:
					printf("Couldn't identify request type\n");
					free_packet_node(packet_node_head);
					return 4;
			}
		}
		else if (input_buffer[0] == '#')
		{
			ADSBPacket packet;
			parse_ADSB_request(&packet, input_buffer);

			Packet_Node *packet_node_buffer = get_packet_node(packet_node_head, packet.id);

			if (!packet_node_buffer)
				add_packet_node(packet, &packet_node_head);
			
			else if (packet_node_buffer->packet.time < packet.time)
				packet_node_buffer->packet = packet;
			
		}
		else
		{
			printf("Invalid input\n");
			free_packet_node(packet_node_head);
			return 2;
		}
	}

	free_packet_node(packet_node_head);
	return 0;
}

int convert_to_time(const int hours, const int minutes)
{
	double total_minutes = hours * 60 + minutes;

	return total_minutes;
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

void parse_ADSB_request(ADSBPacket *packet, const char *input)
{
	int hours, minutes;

	sscanf(input, "#ADS-B:%d,time:%d:%d,N:%lf,E:%lf,alt:%d,head:%lf,speed:%lf\n", &(packet->id), &hours,
	&minutes, &(packet->north), &(packet->east), &(packet->altitude), &(packet->heading), &(packet->speed));

	packet->time = convert_to_time(hours, minutes);
}

void add_packet_node(const ADSBPacket packet, Packet_Node **packet_nodes_head)
{
	Packet_Node *new_packet_node = (Packet_Node *) malloc(sizeof(Packet_Node));

	new_packet_node->packet = packet;
	new_packet_node->next = *packet_nodes_head;

	*packet_nodes_head = new_packet_node;
}

void free_packet_node(Packet_Node *packet_node_head)
{
	while(packet_node_head)
	{
		Packet_Node *temp = packet_node_head;
		packet_node_head = packet_node_head->next;

		free(temp);
	}
}

Packet_Node *get_packet_node(Packet_Node *packet_node_head, int id)
{
	while (packet_node_head)
	{
		if (packet_node_head->packet.id == id)
			return packet_node_head;
		
		packet_node_head = packet_node_head->next;
	}
	return NULL;
}

double to_radians(double deg)
{
	return deg * (PI / 180.0);
}

// Checks if point (x, y) -> (pn, pe) is within a 350km radius, return 0 if in airspace, 1 if not
int in_airspace(double pn, double pe)
{
	return ((pn * pn) + (pe * pe)) > (AIRSPACE_RADIUS * AIRSPACE_RADIUS);
}

int get_est_position(Packet_Node *packet_node_head, Time_Request request, double *est_pos_n, double *est_pos_e)
{
	// Check if id exists
	Packet_Node *packet_node;

	if ((packet_node = get_packet_node(packet_node_head, request.id)) == NULL)
		return 1;

	const ADSBPacket packet = packet_node->packet;

	// Get v_n, v_e and est pos at time t_request
	double v_n, v_e, pn_t, pe_t;

	v_n = packet.speed * sin(to_radians(packet.heading));
	v_e = packet.speed * cos(to_radians(packet.heading));

	pn_t = packet.north + v_n * (request.time - packet.time);
	pe_t = packet.east + v_e * (request.time - packet.time);

	// Check if pn_t || pe_t is in airspace
	if(in_airspace(pn_t, pe_t))
		return 2;

	// Return pass and set pointers to pos
	*est_pos_n = pn_t;
	*est_pos_e = pe_t;

	return 0;
}
