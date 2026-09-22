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

#endif