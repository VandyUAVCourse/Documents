#include "Mavlink_Reactor.h"
#include <utility>

Mavlink_Reactor::Mavlink_Reactor (std::string serialPortDevice)
    : sw (serialPortDevice)
{ }

void Mavlink_Reactor::handlerRegister ( uint8_t msgId, Mavlink_Handler* mh)
{
    handlerMap_.insert ( std::pair <uint8_t, Mavlink_Handler*> (msgId, mh ));
}

void Mavlink_Reactor::react ()
{
        mavlink_message_t rcvMsg;
        mavlink_status_t rcvStatus;
        bool notParsed = true;
        int readBytes = sw.read();
        int rcvBufferIndex(0);
        std::vector <uint8_t> rcvBuffer(70); // RM DEBUG
    
        while (sw.size () > 0 && notParsed) {

            uint8_t byte(sw.get());
            rcvBuffer[rcvBufferIndex++] = byte;
            

            if(mavlink_parse_char(MAVLINK_COMM_0, byte, &rcvMsg, &rcvStatus)) {
                notParsed = false;
                //TODO RM DEBUG
                std::cout << "RECIEVE:" << std::endl;
                for (int i = 0; i < rcvBufferIndex; ++i) {
                    printf("%02x", (unsigned char)rcvBuffer[i]);
                 
                }
                std::cout << std::endl;
                // END RM DEBUG

                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSGID: %d\n",
                        rcvMsg.sysid, rcvMsg.compid, rcvMsg.len, rcvMsg.msgid);

                handlerMap_.at(rcvMsg.msgid)->handle(rcvMsg);

            }
    }
}

void Mavlink_Reactor::send (uint16_t len, uint8_t* buf)
{
    sw.send (len, buf);
}
