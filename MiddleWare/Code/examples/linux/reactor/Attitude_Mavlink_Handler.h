#ifndef ATTITUDE_MAVLINK_HANDLER_H
#define ATTITUDE_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Attitude_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Attitude_Mavlink_Handler.cpp"

#endif



