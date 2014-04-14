
#ifndef SERIAL_WRAPPER_H
#define SERIAL_WRAPPER_H

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

template<int BUFFER_SIZE>
class Serial_Wrapper {
    public:
        //Constructor
        //@param serial_port_device The name of the serial port
        //@param buad The baud rate of the serial port
        //@param charsize The character size to send over the serial port
        //@param parity The number of parity bits to set
        //@param stopbits The number of stop bits to set
        //@param flow_control What kind of flow control to set
        //  TODO: Insert Types for all parameters
        //
        //  @ Description:
        //  Opens a serial port with the specififed name. Optional parameters are set to defaults below
        Serial_Wrapper (string serial_port_device, 
                        LibSerial::SerialStreamBuf::BaudRateEnum baud = LibSerial::SerialStreamBuf::BAUD_9600,
                        LibSerial::SerialStreamBuf::CharSizeEnum charsize = CHAR_SIZE_8,
                        LibSerial::SerialStreamBuf::ParityEnum parity = LibSerial::SerialStreamBuf::PARITY_NONE,
                        int stopbits = 1,
                        LibSerial::SerialStreamBuf::FlowControlEnum flow_control = LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);

        // send
        //@param  msg The message to be sent over the serial port
        //@param  len The length of the message to be sent
        //@param  buf The buffer containing the message to send
        //
        // @description: Sends the message over the serial port
        //
        // TODO: buf - should we templatize the length? How do we handle that?
        // TODO: Better to omit msg entirely? Probably yes, just send in the buffer
        void send ( uint16_t len, uint8_t buf[BUFFER_SIZE]);

        // read
        //
        // @ Description
        // Reads all available data off of serial port into a buffer and returns the number of 
        // available bytes
        int read ();

        // get
        //@param  size: The number of bytes to recieve from the recieving buffer
        //
        // @ Description
        // Returns an array of 'size' bytes, off of a FIFO buffer of data recieved from 
        // the serial port
        //
        // @ Throws
        // Throws an exception TODO: What exception?
        // if more bytes are requested than are available
        std::Array get(int size);

        // size
        // @ Params:
        // none
        //
        // @ Description
        // Returns the number of bytes available to read off of the FIFO buffer
        int size ();

        // Destructor
        //
        // @ Description
        // Closes the serial port.
        ~Serial_Wrapper();

        private:
            std::queue <uint8_t> rcv_buffer;

            SerialStream serial_port;
};

#endif
