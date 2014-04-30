#ifndef VFR_HUD_MAVLINK_HANDLER_H
#define VFR_HUD_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Vfr_Hud_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Vfr_Hud_Mavlink_Handler.cpp"

#endif



