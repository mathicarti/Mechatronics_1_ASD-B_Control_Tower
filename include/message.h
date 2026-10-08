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
double to_m(double km);
double to_km(double m);
double to_radians(double deg);
int solve_quadratic(double a, double b, double c, double *root_1, double *root_2);

void handle_ADSB_packet(char *input, Packet_Node **packet_node_head);
void parse_ADSB_request(ADSBPacket *packet, const char *input);

void add_packet_node(const ADSBPacket packet, Packet_Node **packet_node_head);
void free_packet_node(Packet_Node *packet_node_head);
Packet_Node *get_packet_node(Packet_Node *packet_node_head, int id);

int handle_time_request(char *input, Packet_Node *packet_node_head);
void parse_time_request(Time_Request *request, char *input);

void handle_est_pos(Time_Request request, Packet_Node *packet_node_head);
int get_est_pos(Time_Request request, Packet_Node *packet_node_head, double *north_pos, double *east_pos);

void handle_num_contacts(Time_Request request, Packet_Node *packet_node_head);
int get_num_contacts(int t_check, Packet_Node *packet_node_head);

void handle_check_separation(Time_Request request, Packet_Node *packet_node_head);
int find_first_separation_issue(Packet_Node *packet_node_head, ADSBPacket packet, int t_check, double minimum_sep_distance, double *dt_issue);
int check_pair_for_issue(int t_check, ADSBPacket a, ADSBPacket b, double minimum_sep_distance, double *dt_issue);

void estimate_position(int t_check, ADSBPacket packet, double *north_pos, double *east_pos);
void compute_velocity(ADSBPacket packet, double *north_vel, double *east_vel);
int in_airspace(double north_pos, double east_pos);

#endif