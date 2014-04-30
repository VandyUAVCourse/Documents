#ifndef RC_CHANNELS_RAW_MAVLINK_HANDLER_H
#define RC_CHANNELS_RAW_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Rc_Channels_Raw_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Rc_Channels_Raw_Mavlink_Handler.cpp"

#endif



