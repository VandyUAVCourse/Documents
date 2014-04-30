#include "Mavlink_Handler.h"
#include "Mission_Current_Mavlink_Handler.h"

void Mission_Current_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode( &msg, &mission_current);
    /* printf("\nMission_Current Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         mission_current.seq); */
}

uint8_t Mission_Current_Mavlink_Handler::id ()
{
    return 42;
}
