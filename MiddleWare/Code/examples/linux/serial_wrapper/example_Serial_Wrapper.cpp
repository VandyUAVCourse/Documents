#include <mavlink.h>
#include <array>

#include "Serial_Wrapper.h"

//TODO: RM DEBUG
#include <iostream>

#define MAV_MESSAGE_SIZE 17

int main (int argc, char** argv) {

    Serial_Wrapper <MAV_MESSAGE_SIZE> sw("/dev/ttyACM0");

    uint8_t sendbuf[MAV_MESSAGE_SIZE];
    mavlink_message_t msg;
    uint16_t len;
    mavlink_status_t status;

    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER,
            MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
            0, MAV_STATE_ACTIVE);

    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
    sleep(1);

    sw.send(len, sendbuf);
    sw.send(len, sendbuf);
    sw.send(len, sendbuf);

    sleep(1);

    sw.read();

    while(sw.size() < 1) {
        sleep(1);
        sw.send(len, sendbuf);
        sw.read();
        std::cout << "Test" << std::endl;
    }

    mavlink_msg_heartbeat_pack(1, 255, &msg, MAV_TYPE_HELICOPTER,
            MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
            0, MAV_STATE_ACTIVE);

    len = mavlink_msg_to_send_buffer(sendbuf, &msg);

    while(sw.size() < 1) {
        sleep(1);
        sw.send(len, sendbuf);
        sw.read();
        std::cout << "Test" << std::endl;
    }

    std::array <uint8_t, MAV_MESSAGE_SIZE> arr(sw.get());

    std::cout << "out of get" << std::endl;
    for ( auto& elem : arr) {
        printf("%02x", (unsigned char)elem);
    }

    if(mavlink_parse_char(MAVLINK_COMM_0, arr[0], &msg, &status)) {

        printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
                msg.sysid, msg.compid, msg.len, msg.msgid);
    }

    sleep(1);
    sw.send(len, sendbuf);
    std::cout << std::endl;

    sleep(1);
    sw.read();

    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
            msg.sysid, msg.compid, msg.len, msg.msgid);
}
