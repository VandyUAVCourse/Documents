#include <mavlink.h>
#include <vector>

#include "Serial_Wrapper.h"

//TODO: RM DEBUG
#include <iostream>

#define MAV_MESSAGE_SIZE 17


int main (int argc, char** argv) {

    Serial_Wrapper sw("/dev/ttyACM0");

    std::vector <uint8_t> rcvBuffer(70); //To store the contents of recieved message for debugging
    uint16_t len;
    int readBytes = 0;

    sleep(1);

    while(1) {
        uint8_t sendbuf[MAV_MESSAGE_SIZE];
        mavlink_message_t msg;
        mavlink_message_t rcvMsg;
        mavlink_status_t rcvStatus;
        bool notParsed = true;

        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER,
                MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
                0, MAV_STATE_ACTIVE);

        len = mavlink_msg_to_send_buffer(sendbuf, &msg);
        std::cout << "SEND:" << std::endl;
        for ( auto& elem : sendbuf) {
            printf("%02x", (unsigned char)elem);
        }
        std::cout << std::endl;

        sw.send(len, sendbuf);

        sleep(1);
        readBytes = sw.read();
        std::cout << "Read in " << readBytes << " bytes" << std::endl;
        int rcvBufferIndex(0);

        while (sw.size () > 0 && notParsed) {

            uint8_t byte(sw.get());
            rcvBuffer[rcvBufferIndex++] = byte;
            

            if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rcvMsg, &rcvStatus)) {
                notParsed = false;
                std::cout << "RECIEVE:" << std::endl;
                for (int i = 0; i < rcvBufferIndex; ++i) {
                    printf("%02x", (unsigned char)rcvBuffer[i]);
                 
                }
                std::cout << std::endl;

                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
                        rcvMsg.sysid, rcvMsg.compid, rcvMsg.len, rcvMsg.msgid);

                if (rcvMsg.msgid == 0) {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode( &rcvMsg, &heartbeat);
                    printf("\nHeartbeat Message: CUSTOM_MODE: %u TYPE: %d AUTOPILOT: %u BASE_MODE: %u SYSTEM_STATUS: %u MAVLINK_VERSION: %u\n", 
                            heartbeat.custom_mode,
                            heartbeat.type,
                            heartbeat.autopilot ,
                            heartbeat.base_mode ,
                            heartbeat.system_status,
                            heartbeat.mavlink_version);
                }

            }
        }
    }
}
