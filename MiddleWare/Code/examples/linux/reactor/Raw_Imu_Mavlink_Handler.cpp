#include "Mavlink_Handler.h"
#include "Raw_Imu_Mavlink_Handler.h"

void Raw_Imu_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_raw_imu_t raw_imu;
    mavlink_msg_raw_imu_decode( &msg, &raw_imu);
    /* printf("\nRaw_Imu Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         raw_imu.time_usec, */
    /*         raw_imu.xacc, */
    /*         raw_imu.yacc, */
    /*         raw_imu.zacc, */
    /*         raw_imu.xgyro, */
    /*         raw_imu.ygyro, */
    /*         raw_imu.zgyro, */
    /*         raw_imu.xmag, */
    /*         raw_imu.ymag, */
    /*         raw_imu.zmag); */

}

uint8_t Raw_Imu_Mavlink_Handler::id ()
{
    return 27;
}
