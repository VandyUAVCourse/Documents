#ifndef SYS_STATUS_MAVLINK_HANDLER_H
#define SYS_STATUS_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Sys_Status_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Sys_Status_Mavlink_Handler.cpp"

#endif



