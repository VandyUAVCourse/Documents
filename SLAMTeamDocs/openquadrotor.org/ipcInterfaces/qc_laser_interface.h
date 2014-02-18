#ifndef _QC_LASER_INTERFACE_H_
#define _QC_LASER_INTERFACE_H_

#include <iostream> 
#include <ipc/ipc.h> 
#include <utils_ipc/colormakros.h> 
#include <ipcMessages/qc_laser_messages.h>


int qc_laser_subscribe_laser1_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_laser_publish_laser1_message (qc_laser_laser_message* msg); 
int qc_laser_unsubscribe_laser1_message (HANDLER_TYPE handler); 



///********* NEXT MESSAGE ***********///  


int qc_laser_subscribe_laser2_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_laser_publish_laser2_message (qc_laser_laser_message* msg); 
int qc_laser_unsubscribe_laser2_message (HANDLER_TYPE handler); 



#endif // _QC_LASER_INTERFACE_H_
