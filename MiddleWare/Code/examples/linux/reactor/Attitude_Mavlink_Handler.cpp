#include "Mavlink_Handler.h"
#include "Attitude_Mavlink_Handler.h"

void Attitude_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode( &msg, &attitude);
    /* printf("\nAttitude Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         attitude.time_boot_ms, */
    /*         attitude.roll, */
    /*         attitude.pitch, */
    /*         attitude.yaw, */
    /*         attitude.rollspeed, */
    /*         attitude.pitchspeed, */
    /*         attitude.yawspeed); */

}

uint8_t Attitude_Mavlink_Handler::id ()
{
    return 30;
}
