#ifndef GPS_RAW_INT_MAVLINK_HANDLER_H
#define GPS_RAW_INT_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Gps_Raw_Int_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Gps_Raw_Int_Mavlink_Handler.cpp"

#endif



