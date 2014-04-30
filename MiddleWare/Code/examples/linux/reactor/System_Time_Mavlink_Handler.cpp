#include "Mavlink_Handler.h"
#include "System_Time_Mavlink_Handler.h"

void System_Time_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_system_time_t system_time;
    mavlink_msg_system_time_decode( &msg, &system_time);
    /* printf("\nSys_Status Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         system_time.time_unix_usec, */
    /*         system_time.time_boot_ms); */

}

uint8_t System_Time_Mavlink_Handler::id ()
{
    return 2;
}
