#ifndef MISSION_CURRENT_MAVLINK_HANDLER_H
#define MISSION_CURRENT_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Mission_Current_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Mission_Current_Mavlink_Handler.cpp"

#endif



