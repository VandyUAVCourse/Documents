#include "Mavlink_Reactor.h"
#include "Mavlink_Handler.h"
#include "System_Time_Mavlink_Handler.h"
#include "Heartbeat_Mavlink_Handler.h"
#include "Vfr_Hud_Mavlink_Handler.h"
#include "Heartbeat_Mavlink_Handler.h"
#include "Gps_Raw_Int_Mavlink_Handler.h"
#include "Servo_Output_Raw_Mavlink_Handler.h"
#include "Mission_Current_Mavlink_Handler.h"
#include "Nav_Controller_Output_Mavlink_Handler.h"
#include "Scaled_Pressure_Mavlink_Handler.h"
#include "Attitude_Mavlink_Handler.h"
#include "Raw_Imu_Mavlink_Handler.h"
#include "Global_Position_Int_Mavlink_Handler.h"
#include "Sys_Status_Mavlink_Handler.h"
#include "Rc_Channels_Raw_Mavlink_Handler.h"

const char * PORT_NAME = "/dev/ttyACM0";
const uint MAV_MESSAGE_SIZE = 17;

int main (int argc, char ** argv) 
{
    Mavlink_Reactor mr ( PORT_NAME );

    mr.handlerRegister ( System_Time_Mavlink_Handler        ::id (), new System_Time_Mavlink_Handler);
    mr.handlerRegister ( Heartbeat_Mavlink_Handler          ::id (), new Heartbeat_Mavlink_Handler);
    mr.handlerRegister ( Vfr_Hud_Mavlink_Handler            ::id (), new Vfr_Hud_Mavlink_Handler);
    mr.handlerRegister ( Heartbeat_Mavlink_Handler          ::id (), new Heartbeat_Mavlink_Handler);
    mr.handlerRegister ( Gps_Raw_Int_Mavlink_Handler        ::id (), new Gps_Raw_Int_Mavlink_Handler);
    mr.handlerRegister ( Servo_Output_Raw_Mavlink_Handler   ::id (), new Servo_Output_Raw_Mavlink_Handler);
    mr.handlerRegister ( Mission_Current_Mavlink_Handler    ::id (), new Mission_Current_Mavlink_Handler);
    mr.handlerRegister ( Nav_Controller_Output_Mavlink_Handler     ::id (), new Nav_Controller_Output_Mavlink_Handler);
    mr.handlerRegister ( Scaled_Pressure_Mavlink_Handler    ::id (), new Scaled_Pressure_Mavlink_Handler);
    mr.handlerRegister ( Attitude_Mavlink_Handler           ::id (), new Attitude_Mavlink_Handler);
    mr.handlerRegister ( Raw_Imu_Mavlink_Handler            ::id (), new Raw_Imu_Mavlink_Handler);
    mr.handlerRegister ( Global_Position_Int_Mavlink_Handler::id (), new Global_Position_Int_Mavlink_Handler);
    mr.handlerRegister ( Sys_Status_Mavlink_Handler         ::id (), new Sys_Status_Mavlink_Handler);
    mr.handlerRegister ( Rc_Channels_Raw_Mavlink_Handler    ::id (), new Rc_Channels_Raw_Mavlink_Handler);

    uint8_t sendbuf[MAV_MESSAGE_SIZE];
    uint16_t len;
    mavlink_message_t msg;

    while (1) {
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER,
                MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
                0, MAV_STATE_ACTIVE);

        len = mavlink_msg_to_send_buffer(sendbuf, &msg);

        std::cout << "SEND:" << std::endl;
        for ( auto& elem : sendbuf) {
            printf("%02x", (unsigned char)elem);
        }
        std::cout << std::endl;

        mr.send(len, sendbuf);

        sleep(1);

        mr.react ();

    }
}
