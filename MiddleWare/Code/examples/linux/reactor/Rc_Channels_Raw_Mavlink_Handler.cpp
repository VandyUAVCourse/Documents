#include "Mavlink_Handler.h"
#include "Rc_Channels_Raw_Mavlink_Handler.h"

void Rc_Channels_Raw_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_rc_channels_raw_t rc_channels_raw;
    mavlink_msg_rc_channels_raw_decode( &msg, &rc_channels_raw);
    /* printf("\nRc_Channels_Raw Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         rc_channels_raw.time_boot_ms, */
    /*         rc_channels_raw.port, */
    /*         rc_channels_raw.chan1_raw, */
    /*         rc_channels_raw.chan2_raw, */
    /*         rc_channels_raw.chan3_raw, */
    /*         rc_channels_raw.chan4_raw, */
    /*         rc_channels_raw.chan5_raw, */
    /*         rc_channels_raw.chan6_raw, */
    /*         rc_channels_raw.chan7_raw, */
    /*         rc_channels_raw.chan8_raw, */
    /*         rc_channels_raw.rssi); */

}

uint8_t Rc_Channels_Raw_Mavlink_Handler::id ()
{
    return 35;
}
