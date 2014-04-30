#ifndef SYSTEM_TIME_MAVLINK_HANDLER_H
#define SYSTEM_TIME_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class System_Time_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "System_Time_Mavlink_Handler.cpp"

#endif



