#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
			if (handle_time_request(input_buffer, packet_node_head) != 0)
				break;
		}
		else if (input_buffer[0] == '#')
		{
			handle_ADSB_packet(input_buffer, &packet_node_head);
		}
		else
		{
			printf("Invalid input, try \"time:<hour>:<minute>,close\"\n");
			break;
		}
	}

	free_packet_node(packet_node_head);
	return 0;
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

int convert_to_time(const int hours, const int minutes)
{
	double total_minutes = hours * 60 + minutes;
	int total_seconds = total_minutes * 60;

	return total_seconds;
}

double to_m(double km)
{
	return km * 1000.0;
}

double to_km(double m)
{
	return m / 1000.0;
}

double to_radians(double deg)
{
	return deg * (PI / 180.0);
}

int solve_quadratic(double a, double b, double c, double *root_1, double *root_2)
{
	// Obtain determinant
	double det = (b*b) - (4*a*c);

	// Obtain roots
	if (det > 0)
	{
		*root_1 = -(b + sqrt(det)) / (2 * a);
		*root_2 = -(b - sqrt(det)) / (2 * a);
	}
	else if (det = 0)
	{
		*root_1 = -b / (2 * a);
		*root_2 = *root_1;
	}
	else if (det < 0)
		return 0;
}
