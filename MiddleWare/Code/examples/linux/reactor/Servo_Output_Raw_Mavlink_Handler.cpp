#include "Mavlink_Handler.h"
#include "Servo_Output_Raw_Mavlink_Handler.h"

void Servo_Output_Raw_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_servo_output_raw_t heartbeat;
    mavlink_msg_servo_output_raw_decode( &msg, &heartbeat);
    /* printf("\nServo_Output_Raw Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         servo_output_raw.time_usec, */
    /*         servo_output_raw.port, */
    /*         servo_output_raw.servo1_raw, */
    /*         servo_output_raw.servo2_raw, */
    /*         servo_output_raw.servo3_raw, */
    /*         servo_output_raw.servo4_raw, */
    /*         servo_output_raw.servo5_raw, */
    /*         servo_output_raw.servo6_raw, */
    /*         servo_output_raw.servo7_raw, */
    /*         servo_output_raw.servo8_raw); */

}

uint8_t Servo_Output_Raw_Mavlink_Handler::id ()
{
    return 36;
}
