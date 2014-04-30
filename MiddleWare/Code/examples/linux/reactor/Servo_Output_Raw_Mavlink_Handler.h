#ifndef SERVO_OUTPUT_RAW_MAVLINK_HANDLER_H
#define SERVO_OUTPUT_RAW_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Servo_Output_Raw_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Servo_Output_Raw_Mavlink_Handler.cpp"

#endif



