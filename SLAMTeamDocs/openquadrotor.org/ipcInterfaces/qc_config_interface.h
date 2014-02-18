#ifndef _QC_CONFIG_INTERFACE_H_
#define _QC_CONFIG_INTERFACE_H_

#include <iostream> 
#include <ipc/ipc.h> 
#include <utils_ipc/colormakros.h> 
#include <ipcMessages/qc_config_messages.h>


int qc_config_subscribe_request_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_config_publish_request_message (qc_config_request_message* msg); 
int qc_config_unsubscribe_request_message (HANDLER_TYPE handler); 



///********* NEXT MESSAGE ***********///  


int qc_config_subscribe_parameter_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_config_publish_parameter_message (qc_config_parameter_message* msg); 
int qc_config_unsubscribe_parameter_message (HANDLER_TYPE handler); 



#endif // _QC_CONFIG_INTERFACE_H_
