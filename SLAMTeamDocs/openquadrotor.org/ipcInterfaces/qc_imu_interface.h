#ifndef _QC_IMU_INTERFACE_H_
#define _QC_IMU_INTERFACE_H_

#include <iostream> 
#include <ipc/ipc.h> 
#include <utils_ipc/colormakros.h> 
#include <ipcMessages/qc_imu_messages.h>


int qc_imu_subscribe_imu_message (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); 
int qc_imu_publish_imu_message (qc_imu_imu_message* msg); 
int qc_imu_unsubscribe_imu_message (HANDLER_TYPE handler); 



#endif // _QC_IMU_INTERFACE_H_
