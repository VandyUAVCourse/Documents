//TODO: RM DEBUG
#include <iostream>

//Constructor
//@param serialPortDevice The name of the serial port
//@param buad The baud rate of the serial port
//@param charsize The character size to send over the serial port
//@param parity The number of parity bits to set
//@param stopbits The number of stop bits to set
//@param flow_control What kind of flow control to set
//  TODO: Insert Types for all parameters
//
//  @ Description:
//  Opens a serial port with the specififed name. Optional parameters are set to defaults below
template<int BUFFER_SIZE>
Serial_Wrapper<BUFFER_SIZE>::Serial_Wrapper (std::string serialPortDevice, 
        LibSerial::SerialStreamBuf::BaudRateEnum baud,
        LibSerial::SerialStreamBuf::CharSizeEnum charsize,
        LibSerial::SerialStreamBuf::ParityEnum parity,
        int stopbits,
        LibSerial::SerialStreamBuf::FlowControlEnum flow_control)
    // Default Initializers are OK 
{
    serialPort_.Open(serialPortDevice);
    serialPort_.SetBaudRate(baud);
    serialPort_.SetCharSize(charsize);
    serialPort_.SetParity(parity);
    serialPort_.SetNumOfStopBits(stopbits);
    serialPort_.SetFlowControl(flow_control);
    if(!serialPort_.good()) 
        throw std::runtime_error ("Serial_Wrapper::Serial_Wrapper Could not initialize " + serialPortDevice);
}

// send
//@param  msg The message to be sent over the serial port
//@param  len The length of the message to be sent
//@param  buf The buffer containing the message to send
//
// @description: Sends the message over the serial port
//
template<int BUFFER_SIZE>
void Serial_Wrapper<BUFFER_SIZE>::send ( uint16_t len, uint8_t* buf)
{
    serialPort_.write((char*)buf, len);
}

// read
//
// @ Description
// Reads all available data off of serial port into a buffer and returns the number of 
// available bytes
template <int BUFFER_SIZE>
int Serial_Wrapper<BUFFER_SIZE>::read () 
{
    int bytesRcvd(0);
    char * byte_buffer = new char [BUFFER_SIZE];
    std::fill(byte_buffer, byte_buffer + BUFFER_SIZE, 0);
    char trash;

    sleep(1);
    if (serialPort_.rdbuf()->in_avail() > 0) {
        while (serialPort_.rdbuf()->in_avail() > 0) {
            byte_buffer[bytesRcvd] = serialPort_.peek();
            serialPort_.get(trash);
            ++bytesRcvd;
            // uncomment for proof that correct thing is recieved
            //printf("%02x", (unsigned char)trash);
        }
        rcvBuffer_.push(byte_buffer);
    }

    return bytesRcvd;
}

// get
//
// @ Description
// Returns an array of 'size' bytes, off of a FIFO buffer of data recieved from 
// the serial port
//
// @ Throws
// Throws an exception TODO: What exception?
// if more bytes are requested than are available
template <int BUFFER_SIZE>
std::array <uint8_t, BUFFER_SIZE> Serial_Wrapper<BUFFER_SIZE>::get ()
{
    if (size () < 1)
        throw std::underflow_error("Serial_Wrapper::get () Not enough bytes in queue to pack a message");

    std::array <uint8_t, BUFFER_SIZE> arr;

    char * getByteBuf = rcvBuffer_.front(); 

    rcvBuffer_.pop();

    for (int i = 0; i < BUFFER_SIZE; ++i) {
        printf("%02x", (unsigned char)getByteBuf[i]);
        arr[i] = (uint8_t) getByteBuf[i];
    }

    return arr;
}

// size
// @ Params:
// none
//
// @ Description
// Returns the number of bytes available to read off of the FIFO buffer
template <int BUFFER_SIZE>
int Serial_Wrapper<BUFFER_SIZE>::size ()
{
    return rcvBuffer_.size();
}

// Destructor
//
// @ Description
// Closes the serial port.
template <int BUFFER_SIZE>
Serial_Wrapper<BUFFER_SIZE>::~Serial_Wrapper()
{
    serialPort_.Close(); 
}


