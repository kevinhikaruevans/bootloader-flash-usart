#ifndef EVENTQUEUE2_H
#define EVENTQUEUE2_H

#include "mbed-os/events/EventQueue.h"

/**
 * A class extending the EventQueue to include a "has_next" functionality
 * 
 * @author Kevin Evans
 */
class EventQueue2 : public EventQueue {
public:
    EventQueue2(unsigned size)
        : EventQueue(size) {}

    /**
     * Returns true if the queue contains more events after the currently running one.
     */
    bool has_next() {
        equeue_mutex_lock(&this->_equeue.queuelock);
        bool hasNext = this->_equeue.queue != NULL;
        equeue_mutex_unlock(&this->_equeue.queuelock);

        return hasNext;
    }
};
#endif