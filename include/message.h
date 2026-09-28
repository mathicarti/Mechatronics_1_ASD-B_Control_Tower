#ifndef MESSAGE_H
#define MESSAGE_H

typedef struct 
{
    int id;
    double time;
    double north;
    double east;
    int altitude;
    double heading;
    double speed;
} ADSBPacket;

typedef struct Node
{
    ADSBPacket packet;
    struct Node *next;
    struct Node *previous;
} Packet_Node;

typedef enum
{
    CLOSING,
    EST_POS,
    NUM_CONTACTS,
    CHECK_SEPARATION
} Request_ID;

typedef struct
{
    double time;
    Request_ID request_ID;
    int aircraft_id;
    double minimum_sep_distance;
} Time_Request;

double convert_to_time(int hours, int minutes);

#endif