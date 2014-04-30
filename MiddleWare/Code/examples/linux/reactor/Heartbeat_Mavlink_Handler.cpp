#include "Mavlink_Handler.h"
#include "Heartbeat_Mavlink_Handler.h"

void Heartbeat_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode( &msg, &heartbeat);
    printf("\nHeartbeat Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", 
            heartbeat.custom_mode,
            heartbeat.type,
            heartbeat.autopilot ,
            heartbeat.base_mode ,
            heartbeat.system_status,
            heartbeat.mavlink_version);

}

uint8_t Heartbeat_Mavlink_Handler::id ()
{
    return 0;
}
