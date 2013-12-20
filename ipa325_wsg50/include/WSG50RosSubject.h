#pragma once

#include <map>
#include <set>

class WSG50RosObserver;

class WSG50RosSubject
{
public:

    /*
     *  attach a new observer for a certain msg Id
     */
    void Attach(WSG50RosObserver * observer_, unsigned int msgId_);
    void Detach(WSG50RosObserver * observer_, unsigned int msgId_);

protected:
    // map where the observers are stored by the keys they are assigned to.
    std::map< unsigned int, std::set<WSG50RosObserver * > > _observers;
};
