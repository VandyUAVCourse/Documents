#include "Mavlink_Handler.h"
#include "Statustext_Mavlink_Handler.h"

void Statustext_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode( &msg, &statustext);
    printf("\nStatustext Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", 
            statustext.custom_mode,
            statustext.type,
            statustext.autopilot ,
            statustext.base_mode ,
            statustext.system_status,
            statustext.mavlink_version);

}

uint8_t Statustext_Mavlink_Handler::id ()
{
    return 253;
}
