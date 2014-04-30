#include <mavlink.h>
#include "Mavlink_Handler.h"
#include "Nav_Controller_Output_Mavlink_Handler.h"

void Nav_Controller_Output_Mavlink_Handler::handle (mavlink_message_t &msg)
{
    mavlink_nav_controller_output_t nav_controller_output;
    mavlink_msg_nav_controller_output_decode( &msg, &nav_controller_output);
    /* printf("\nNav_Controller Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", */ 
    /*         nav_controller.nav_roll, */
    /*         nav_controller.nav_pitch, */
    /*         nav_controller.nav_bearing, */
    /*         nav_controller.target_bearing, */
    /*         nav_controller.wp_dist, */
    /*         nav_controller.alt_error, */
    /*         nav_controller.aspd_error, */
    /*         nav_controller.xtrack_error); */

}

uint8_t Nav_Controller_Output_Mavlink_Handler::id ()
{
    return 62;
}
