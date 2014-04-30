#ifndef GLOBAL_POSITION_INT_MAVLINK_HANDLER_H
#define GLOBAL_POSITION_INT_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Global_Position_Int_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Global_Position_Int_Mavlink_Handler.cpp"

#endif



