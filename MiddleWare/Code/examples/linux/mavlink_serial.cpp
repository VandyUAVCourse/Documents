#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <iostream>
#include <iomanip>
#include <SerialStream.h>
#include <fstream>
#include <mavlink.h>


#define BUFFER_LENGTH 2041

using namespace LibSerial;

uint64_t microsSinceEpoch();

int main(int argc, char* argv[]) {
       
  uint8_t buf[BUFFER_LENGTH];
  mavlink_message_t msg;
  uint16_t len;  
  char next_byte;
  mavlink_status_t status;

  char* SERIAL_PORT_DEVICE = "/dev/ttyACM0";
  std::cout << "serial port name initialized" << std::endl;
  
  
  // Set the serial port device
  if (argc == 2) {
    strcpy(SERIAL_PORT_DEVICE, argv[1]);
  }
  std::cout << "SERIAL PORT: " << SERIAL_PORT_DEVICE << std::endl;
 // terminal close file
  SerialStream serial_port;
  std::cout << "port initialized" << std::endl;

  serial_port.Open(SERIAL_PORT_DEVICE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not open serial port: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Opened..." << std::endl;

  serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Baud Rate (115200): "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Baud set..." << std::endl;

  serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8); 
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Char size to 8: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Char size set..." << std::endl;

  serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Error disabling parity: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Parity set..." << std::endl;
	
  serial_port.SetNumOfStopBits(1);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could set # stop bits to 1: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Stop bits set..." << std::endl;

  serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Flow Control to none: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
  std::cout << "Flow control set..." << std::endl;
  std::cout << "----Setup complete----" << std::endl;

  int msg_send_num = 0;
  while(1) {
		
    /*Send Heartbeat */
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER,
		MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
		0, MAV_STATE_ACTIVE);

    len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_port.write((char*)&buf, len);
    std::cout << "len: " << len << std::endl;
    
    std::cout << "SEND: ";
    char temp;    
    for(int i = 0; i < len; ++i) {
      temp = buf[i];
      printf("%02x", (unsigned char)temp);
    }
    std::cout << std::endl;

    memset(buf, 0, BUFFER_LENGTH);

    int i = 0;
    if(serial_port.rdbuf()->in_avail() > 0) {
      while(serial_port.rdbuf()->in_avail() > 0) {
	serial_port.get(next_byte);
	buf[i] = (uint8_t)next_byte;
	++i;
      }
    }
    
    std::cout << "RECV: ";
    for(int j = 0; j < i; ++j) {
      temp = buf[j];
      printf("%02x", (unsigned char)temp);

      if(mavlink_parse_char(MAVLINK_COMM_0, buf[j], &msg, &status)) {

	printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
	       msg.sysid, msg.compid, msg.len, msg.msgid);
      }
    }
    
    std::cout << std::endl;
    
    memset(buf, 0, BUFFER_LENGTH);
    sleep(1);
  }
}

uint64_t microsSinceEpoch() {
	
  struct timeval tv;
	
  uint64_t micros = 0;
	
  gettimeofday(&tv, NULL);  
  micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
  return micros;
}
