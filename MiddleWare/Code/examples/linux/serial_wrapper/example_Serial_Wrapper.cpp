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
	sw.send(len, sendbuf);

	sleep(1);

	sw.read();

	while(sw.size() < 17) {
		sleep(1);
		sw.send(len, sendbuf);
		sw.read();
		std::cout << "Test" << std::endl;
	}

	std::array <uint8_t, MAV_MESSAGE_SIZE> arr(sw.get());
	if(mavlink_parse_char(MAVLINK_COMM_0, arr[0], &msg, &status)) {

	printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
	       msg.sysid, msg.compid, msg.len, msg.msgid);
	}
}
