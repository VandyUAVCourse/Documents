#ifndef MAVLINK_REACTOR_H
#define MAVLINK_REACTOR_H

#include <string>
#include <map>
#include <mavlink.h>
#include <utility>
#include "Mavlink_Handler.h"
#include "Serial_Wrapper.h"

class Mavlink_Reactor {
    public:
        // TODO
        // ctor that initializes the Serial_Wrapper
        Mavlink_Reactor (std::string serialPortDevice);

        // handlerRegister
        // @brief Registers handlers for different message types
        //
        // @param   msgId   The mavlink_message_t.msgId the Handler should register to
        // @param   Mavlink_Handler   The handler for that type of message
        void handlerRegister (uint8_t msgId, Mavlink_Handler* mh);

        // send
        // @brief   Puts a message into the reactor to be handled by the appropriate handler
        //
        // @param   len     Length of the message in bytes
        // @param   buf     The mesasge to be sent
        void send (uint16_t len, uint8_t* buf);

        // react
        // @brief   The 'main loop' of the reactor. Is blocking
        //
        // Checks the input source for messages and reacts to them
        void react ();

    private:
        // handlerMap_ 
        // @brief Contains all the handlers to act on the incoming mavlink messages
        std::map <uint8_t, Mavlink_Handler*> handlerMap_;

        Serial_Wrapper sw;
};

#include "Mavlink_Reactor.cpp"

#endif

