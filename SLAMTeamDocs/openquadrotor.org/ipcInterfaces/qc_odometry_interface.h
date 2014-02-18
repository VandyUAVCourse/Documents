#ifndef _QC_ODOMETRY_INTERFACE_H_
#define _QC_ODOMETRY_INTERFACE_H_

#include <iostream> 
#include <ipc/ipc.h> 
#include <utils_ipc/colormakros.h> 
#include <ipcMessages/qc_odometry_messages.h>


int qc_odometry_subscribe_odometry_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_odometry_publish_odometry_message (qc_odometry_odometry_message* msg); 
int qc_odometry_unsubscribe_odometry_message (HANDLER_TYPE handler); 



///********* NEXT MESSAGE ***********///  


int qc_odometry_subscribe_odometry_laserpoints_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_odometry_publish_odometry_laserpoints_message (qc_odometry_odometry_laserpoints_message* msg); 
int qc_odometry_unsubscribe_odometry_laserpoints_message (HANDLER_TYPE handler); 



///********* NEXT MESSAGE ***********///  


int qc_odometry_subscribe_velocity_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_odometry_publish_velocity_message (qc_odometry_velocity_message* msg); 
int qc_odometry_unsubscribe_velocity_message (HANDLER_TYPE handler); 



#endif // _QC_ODOMETRY_INTERFACE_H_
