#ifndef RAW_IMU_MAVLINK_HANDLER_H
#define RAW_IMU_MAVLINK_HANDLER_H

#include "Mavlink_Handler.h"


class Raw_Imu_Mavlink_Handler : public Mavlink_Handler {
    public:
        virtual void handle (mavlink_message_t &msg);

        static uint8_t id ();
};

#include "Raw_Imu_Mavlink_Handler.cpp"

#endif



