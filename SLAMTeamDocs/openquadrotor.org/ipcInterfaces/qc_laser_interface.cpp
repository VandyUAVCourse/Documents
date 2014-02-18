#include <ipcInterfaces/qc_laser_interface.h>


int qc_laser_subscribe_laser1_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_laser_laser1_message..." << std::flush;
  if (IPC_subscribe(QC_LASER_LASER1_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_LASER_LASER1_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_laser_publish_laser1_message (qc_laser_laser_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_LASER_LASER1_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_laser_laser1_message..." << std::flush;
    if (IPC_defineMsg(QC_LASER_LASER1_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_LASER_LASER1_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_LASER_LASER1_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_laser_laser1_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_laser_unsubscribe_laser1_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_laser_laser1_message..." << std::flush;
  if (IPC_unsubscribe(QC_LASER_LASER1_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


///********* NEXT MESSAGE ***********///  


int qc_laser_subscribe_laser2_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_laser_laser2_message..." << std::flush;
  if (IPC_subscribe(QC_LASER_LASER2_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_LASER_LASER2_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_laser_publish_laser2_message (qc_laser_laser_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_LASER_LASER2_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_laser_laser2_message..." << std::flush;
    if (IPC_defineMsg(QC_LASER_LASER2_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_LASER_LASER2_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_LASER_LASER2_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_laser_laser2_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_laser_unsubscribe_laser2_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_laser_laser2_message..." << std::flush;
  if (IPC_unsubscribe(QC_LASER_LASER2_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


