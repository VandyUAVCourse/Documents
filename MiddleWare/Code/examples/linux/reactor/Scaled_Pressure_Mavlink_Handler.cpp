#include "Mavlink_Handler.h"
#include "Scaled_Pressure_Mavlink_Handler.h"

void Scaled_Pressure_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_scaled_pressure_t scaled_pressure;
    mavlink_msg_scaled_pressure_decode( &msg, &scaled_pressure);
    /* printf("\nScaled_Pressure Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         scaled_pressure.time_boot_ms, */
    /*         scaled_pressure.press_abs, */
    /*         scaled_pressure.press_diff, */
    /*         scaled_pressure.temperature); */

}

uint8_t Scaled_Pressure_Mavlink_Handler::id ()
{
    return 29;
}
