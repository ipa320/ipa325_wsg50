#pragma once


//#include "WSG50Observer.h"
class WSG50Observer;

//! Response Messages
//typedef struct
//{
//    unsigned short length;
//    unsigned char id;
//    unsigned short status_code;
//    unsigned char *data;
//} TRESPONSE;

class WSG50Subject
{
public:
    // attach ()
    void Attach(WSG50Observer * observer);

protected:
    WSG50Observer * _observer;
};
