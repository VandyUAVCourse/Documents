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
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>
#include <iostream>
#include <SerialStream.h>
#include <fstream>

#define BUFFER_LENGTH 2041

using namespace LibSerial;

uint64_t microsSinceEpoch();

int main(int argc, char* argv[]) {
       
  uint8_t buf[BUFFER_LENGTH];
  mavlink_message_t msg;
  uint16_t len;

  if(argc < 2) { 
    std::cerr << "Must pass in a valid USB port file name [e.g. /dev/ttyS0]" << std::endl;
    return 1;
  }

  char* SERIAL_PORT_DEVICE;
  
      
  // Set the serial port device
  if (argc == 2) {
    strcpy(SERIAL_PORT_DEVICE, argv[1]);
  }
  	
  SerialStream serial_port;
  serial_port.Open(SERIAL_PORT_DEVICE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not open serial port: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }

  serial_port.SetBaudRate(SerialStreamBuf::BAUD_115200);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Baud Rate (11520): "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }

  serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8); 
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Char size to 8: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }

  serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Error disabling parity: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }
	
  serial_port.SetNumOfStopBits(1);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could set # stop bits to 1: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }

  serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
  if(!serial_port.good()) {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
	      << "Error: Could not set Flow Control to none: "
	      << SERIAL_PORT_DEVICE
	      << std::endl;
    exit(1);
  }

	
  while(1) {
		
    /*Send Heartbeat */
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, 
			       MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 
			       0, MAV_STATE_ACTIVE);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serial_port.write((char*)buf, len);    
		
    memset(buf, 0, BUFFER_LENGTH);
    sleep(5);
  }
}


/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch() {
	
  struct timespec time;
	
  uint64_t micros = 0;
	
  clock_gettime(CLOCK_REALTIME, &time);  
  micros = (uint64_t)time.tv_sec * 100000 + time.tv_nsec/1000;
	
  return micros;
}

#else
uint64_t microsSinceEpoch() {
	
  struct timeval tv;
	
  uint64_t micros = 0;
	
  gettimeofday(&tv, NULL);  
  micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
  return micros;
}
#endif
