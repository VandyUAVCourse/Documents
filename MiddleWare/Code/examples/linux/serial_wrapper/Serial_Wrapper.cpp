
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
Serial_Wrapper::Serial_Wrapper (const char* serialPortDevice, 
                                LibSerial::SerialStreamBuf::BaudRateEnum baud = LibSerial::SerialStreamBuf::BAUD_9600,
                                LibSerial::SerialStreamBuf::CharSizeEnum charsize = CHAR_SIZE_8,
                                LibSerial::SerialStreamBuf::ParityEnum parity = LibSerial::SerialStreamBuf::PARITY_NONE,
                                int stopbits = 1,
                                LibSerial::SerialStreamBuf::FlowControlEnum flow_control = LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE)
    // Default Initializers are OK 
{
    _serialPort.Open(serialPortDevice);
    _serialPort.SetBaudRate(baud);
    _serialPort.SetCharSize(charsize);
    _serialPort.SetParity(parity);
    _serialPort.SetNumOfStopBits(stopbits);
    _serialPort.SetFlowControl(flow_control);
    if(!_serialPort.good()) 
        throw std::runtime_error ("Serial_Wrapper::Serial_Wrapper Could not initialize " + serialPortDevice);
}

// send
//@param  msg The message to be sent over the serial port
//@param  len The length of the message to be sent
//@param  buf The buffer containing the message to send
//
// @description: Sends the message over the serial port
//
// TODO: buf - should we templatize the length? How do we handle that?
// TODO: Better to omit msg entirely? Probably yes, just send in the buffer
template<int MESSAGE_SIZE>
void Serial_Wrapper::send ( uint16_t len, uint8_t &buf[MESSAGE_SIZE])
{
    _serialPort.write((char*)&buf, len);
}

// read
//
// @ Description
// Reads all available data off of serial port into a buffer and returns the number of 
// available bytes
int Serial_Wrapper::read () 
{
    int bytesRcvd(0);
    char next_byte(0);
    if (_serialPort.rdbuff()->in_avail() > 0) {
        while (_serialPort.rdbuf()->in_avail() > 0) {
            _serialPort.get(next_byte);
            _rcvBuffer.push( (uint8_t) next_byte);
            ++bytesRcvd;
        }
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
template <int MESSAGE_SIZE>
std::Array <uint8_t, MESSAGE_SIZE> Serial_Wrapper::get ()
{
    if (size < MESSAGE_SIZE)
        throw std::underflow_error("Serial_Wrapper::get () Not enough bytes in queue to pack a message");

    std::Array <uint8_t, MESSAGE_SIZE> array;

    for ( auto& elem : array) {
        elem = _rcvBuffer.front();
        _rcvBuffer.pop();
    }

    return array;
}

// size
// @ Params:
// none
//
// @ Description
// Returns the number of bytes available to read off of the FIFO buffer
int size ()
{
    return _rcvBuffer.size();
}

// Destructor
//
// @ Description
// Closes the serial port.
~Serial_Wrapper()
{
   _serialPort.Close(); 
}


