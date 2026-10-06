#ifndef MESSAGE_H
#define MESSAGE_H

#define INPUT_BUFFER_SIZE 1024
#define PI 3.141592653589793
#define AIRSPACE_RADIUS 350000.0

typedef struct 
{
    int id;
    int time;
    double north;
    double east;
    int altitude;
    double heading;
    double speed;
} ADSBPacket;

typedef struct Packet_Node
{
    ADSBPacket packet;
    struct Packet_Node *next;
} Packet_Node;

typedef enum
{
    UNDEFINED,
    CLOSING,
    EST_POS,
    NUM_CONTACTS,
    CHECK_SEPARATION
} Request_ID;

typedef struct
{
    double time;
    Request_ID request_ID;
    int id;
    double minimum_sep_distance;
} Time_Request;

int convert_to_time(const int hours, const int minutes);

void parse_time_request(Time_Request *request, char *input);
void parse_ADSB_request(ADSBPacket *packet, const char *input);

void add_packet_node(const ADSBPacket packet, Packet_Node **packet_node_head);
void free_packet_node(Packet_Node *packet_node_head);

Packet_Node *get_packet_node(Packet_Node *packet_node_head, int id);

double to_radians(double deg);
int in_airspace(double pn, double pe);
int get_est_position(Packet_Node *packet_node_head, Time_Request request, double *est_pos_n, double *est_pos_e);

#endif