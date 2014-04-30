#ifndef HEARTBEAT_MAVLINK_HANDLER_H
#define HEARTBEAT_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Heartbeat_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Heartbeat_Mavlink_Handler.cpp"

#endif



