#include <ipcInterfaces/qc_odometry_interface.h>


int qc_odometry_subscribe_odometry_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_odometry_odometry_message..." << std::flush;
  if (IPC_subscribe(QC_ODOMETRY_ODOMETRY_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_ODOMETRY_ODOMETRY_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_odometry_publish_odometry_message (qc_odometry_odometry_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_ODOMETRY_ODOMETRY_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_odometry_odometry_message..." << std::flush;
    if (IPC_defineMsg(QC_ODOMETRY_ODOMETRY_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_ODOMETRY_ODOMETRY_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_ODOMETRY_ODOMETRY_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_odometry_odometry_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_odometry_unsubscribe_odometry_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_odometry_odometry_message..." << std::flush;
  if (IPC_unsubscribe(QC_ODOMETRY_ODOMETRY_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


///********* NEXT MESSAGE ***********///  


int qc_odometry_subscribe_odometry_laserpoints_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_odometry_odometry_laserpoints_message..." << std::flush;
  if (IPC_subscribe(QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_odometry_publish_odometry_laserpoints_message (qc_odometry_odometry_laserpoints_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_odometry_odometry_laserpoints_message..." << std::flush;
    if (IPC_defineMsg(QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_odometry_odometry_laserpoints_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_odometry_unsubscribe_odometry_laserpoints_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_odometry_odometry_laserpoints_message..." << std::flush;
  if (IPC_unsubscribe(QC_ODOMETRY_ODOMETRY_LASERPOINTS_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


///********* NEXT MESSAGE ***********///  


int qc_odometry_subscribe_velocity_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_odometry_velocity_message..." << std::flush;
  if (IPC_subscribe(QC_ODOMETRY_VELOCITY_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_ODOMETRY_VELOCITY_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_odometry_publish_velocity_message (qc_odometry_velocity_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_ODOMETRY_VELOCITY_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_odometry_velocity_message..." << std::flush;
    if (IPC_defineMsg(QC_ODOMETRY_VELOCITY_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_ODOMETRY_VELOCITY_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_ODOMETRY_VELOCITY_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_odometry_velocity_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_odometry_unsubscribe_velocity_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_odometry_velocity_message..." << std::flush;
  if (IPC_unsubscribe(QC_ODOMETRY_VELOCITY_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


