#ifndef NAV_CONTROLLER_OUTPUT_MAVLINK_HANDLER_H
#define NAV_CONTROLLER_OUTPUT_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Nav_Controller_Output_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Nav_Controller_Output_Mavlink_Handler.cpp"

#endif



