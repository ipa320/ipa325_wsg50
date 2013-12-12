 #pragma once

#include "WSG50RosSubject.h"
#include "WSG50Observer.h"      // required for datatype-definition


class WSG50RosObserver
{
public:
    virtual void update(TRESPONSE * response);
    int _name;
};
