#include <ipcInterfaces/qc_flightcontrol_interface.h>


int qc_flightcontrol_subscribe_flightcontrol_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_flightcontrol_flightcontrol_message..." << std::flush;
  if (IPC_subscribe(QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_flightcontrol_publish_flightcontrol_message (qc_flightcontrol_flightcontrol_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_flightcontrol_flightcontrol_message..." << std::flush;
    if (IPC_defineMsg(QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_flightcontrol_flightcontrol_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_flightcontrol_unsubscribe_flightcontrol_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_flightcontrol_flightcontrol_message..." << std::flush;
  if (IPC_unsubscribe(QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


///********* NEXT MESSAGE ***********///  


int qc_flightcontrol_subscribe_offboard_command_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_flightcontrol_offboard_command_message..." << std::flush;
  if (IPC_subscribe(QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_flightcontrol_publish_offboard_command_message (qc_flightcontrol_offboard_command_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_flightcontrol_offboard_command_message..." << std::flush;
    if (IPC_defineMsg(QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_flightcontrol_offboard_command_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_flightcontrol_unsubscribe_offboard_command_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_flightcontrol_offboard_command_message..." << std::flush;
  if (IPC_unsubscribe(QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


