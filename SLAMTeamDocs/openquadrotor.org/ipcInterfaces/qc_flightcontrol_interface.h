#ifndef _QC_FLIGHTCONTROL_INTERFACE_H_
#define _QC_FLIGHTCONTROL_INTERFACE_H_

#include <iostream> 
#include <ipc/ipc.h> 
#include <utils_ipc/colormakros.h> 
#include <ipcMessages/qc_flightcontrol_messages.h>


int qc_flightcontrol_subscribe_flightcontrol_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_flightcontrol_publish_flightcontrol_message (qc_flightcontrol_flightcontrol_message* msg); 
int qc_flightcontrol_unsubscribe_flightcontrol_message (HANDLER_TYPE handler); 



///********* NEXT MESSAGE ***********///  


int qc_flightcontrol_subscribe_offboard_command_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_flightcontrol_publish_offboard_command_message (qc_flightcontrol_offboard_command_message* msg); 
int qc_flightcontrol_unsubscribe_offboard_command_message (HANDLER_TYPE handler); 



#endif // _QC_FLIGHTCONTROL_INTERFACE_H_
