#include "Mavlink_Handler.h"
#include "Vfr_Hud_Mavlink_Handler.h"

void Vfr_Hud_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode( &msg, &vfr_hud);
    /* printf("\nVfr_Hud Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         vfr_hud.airspeed, */
    /*         vfr_hud.groundspeed, */
    /*         vfr_hud.heading, */
    /*         vfr_hud.throttle, */
    /*         vfr_hud.alt, */
    /*         vfr_hud.climb); */

}

uint8_t Vfr_Hud_Mavlink_Handler::id ()
{
    return 74;
}
