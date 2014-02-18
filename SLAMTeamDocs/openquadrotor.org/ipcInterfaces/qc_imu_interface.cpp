#include <ipcInterfaces/qc_imu_interface.h>


int qc_imu_subscribe_imu_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_imu_imu_message..." << std::flush;
  if (IPC_subscribe(QC_IMU_IMU_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_IMU_IMU_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_imu_publish_imu_message (qc_imu_imu_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_IMU_IMU_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_imu_imu_message..." << std::flush;
    if (IPC_defineMsg(QC_IMU_IMU_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_IMU_IMU_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_IMU_IMU_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_imu_imu_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_imu_unsubscribe_imu_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_imu_imu_message..." << std::flush;
  if (IPC_unsubscribe(QC_IMU_IMU_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


