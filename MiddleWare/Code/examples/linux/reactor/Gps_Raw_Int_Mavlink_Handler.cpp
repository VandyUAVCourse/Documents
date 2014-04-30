#include "Mavlink_Handler.h"
#include "Gps_Raw_Int_Mavlink_Handler.h"

void Gps_Raw_Int_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_msg_gps_raw_int_decode( &msg, &gps_raw_int);
    /* printf("\nGps_Raw_Int Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         gps_raw_int.time_usec, */
    /*         gps_raw_int.fix_type, */
    /*         gps_raw_int.lat, */
    /*         gps_raw_int.lon, */
    /*         gps_raw_int.alt, */
    /*         gps_raw_int.eph, */
    /*         gps_raw_int.epv, */
    /*         gps_raw_int.vel, */
    /*         gps_raw_int.cog, */
    /*         gps_raw_int.satellites_visible); */

}

uint8_t Gps_Raw_Int_Mavlink_Handler::id ()
{
    return 24;
}
