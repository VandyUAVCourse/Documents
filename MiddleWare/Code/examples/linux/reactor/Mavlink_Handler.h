#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include <mavlink.h>

class Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg) = 0;
};

#endif


