#ifndef SCALED_PRESSURE_MAVLINK_HANDLER_H
#define SCALED_PRESSURE_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Scaled_Pressure_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Scaled_Pressure_Mavlink_Handler.cpp"

#endif



