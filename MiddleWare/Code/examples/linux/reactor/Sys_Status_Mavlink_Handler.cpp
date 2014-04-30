#include "Mavlink_Handler.h"
#include "Sys_Status_Mavlink_Handler.h"

void Sys_Status_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode( &msg, &sys_status);
    /* printf("\nSys_Status Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         sys_status.onboard_control_sensors_present, */
    /*         sys_status.onboard_control_sensors_enabled, */
    /*         sys_status.onboard_control_sensors_health, */
    /*         sys_status.load, */
    /*         sys_status.voltage_battery, */
    /*         sys_status.current_battery, */
    /*         sys_status.battery_remaining, */
    /*         sys_status.drop_rate_comm, */
    /*         sys_status.errors_comm, */
    /*         sys_status.errors_count1, */
    /*         sys_status.errors_count2, */
    /*         sys_status.errors_count3, */
    /*         sys_status.errors_count4) */



}

uint8_t Sys_Status_Mavlink_Handler::id ()
{
    return 1;
}
