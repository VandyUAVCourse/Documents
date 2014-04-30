#include "Mavlink_Handler.h"
#include "Global_Position_Int_Mavlink_Handler.h"

void Global_Position_Int_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_global_position_int_t global_position_int;
    mavlink_msg_global_position_int_decode( &msg, &global_position_int);
    /* printf("\nGlobal_Position_Int Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         global_position_int.time_boot_ms, */
    /*         global_position_int.lat, */
    /*         global_position_int.lon, */
    /*         global_position_int.alt, */
    /*         global_position_int.relative_alt, */
    /*         global_position_int.vx, */
    /*         global_position_int.vy, */
    /*         global_position_int.vz, */
    /*         global_position_int.hdg); */

}

uint8_t Global_Position_Int_Mavlink_Handler::id ()
{
    return 33;
}
