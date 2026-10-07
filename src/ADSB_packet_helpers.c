#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "message.h"

void handle_ADSB_packet(char *input, Packet_Node **packet_node_head)
{
    ADSBPacket packet;
    parse_ADSB_request(&packet, input);

    Packet_Node *packet_node_buffer = get_packet_node(*packet_node_head, packet.id);

    if (!packet_node_buffer)
        add_packet_node(packet, packet_node_head);
    
    else if (packet_node_buffer->packet.time < packet.time)
        packet_node_buffer->packet = packet;
}

void parse_ADSB_request(ADSBPacket *packet, const char *input)
{
	int hours, minutes;
	double north, east;

	sscanf(input, "#ADS-B:%d,time:%d:%d,N:%lf,E:%lf,alt:%d,head:%lf,speed:%lf\n", &(packet->id), &hours,
	&minutes, &north, &east, &(packet->altitude), &(packet->heading), &(packet->speed));

	packet->north = to_m(north);
	packet->east = to_m(east);

	packet->time = convert_to_time(hours, minutes);
}
