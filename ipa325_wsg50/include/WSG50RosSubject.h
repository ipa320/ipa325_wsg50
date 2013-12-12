#pragma once

#include <map>
#include <set>

class WSG50RosObserver;

class WSG50RosSubject
{
public:

    /*
     *  to be overloaded
     *  attach a new observer for a certain msg Id
     */
    //virtual void Attach(WSG50RosObserver observer_, unsigned int msgId_);
    void Attach(WSG50RosObserver * observer_, unsigned int msgId_);
    void Detach(WSG50RosObserver * observer_, unsigned int msgId_);

protected:
    std::map< unsigned int, std::set<WSG50RosObserver * > > _observers;
};
