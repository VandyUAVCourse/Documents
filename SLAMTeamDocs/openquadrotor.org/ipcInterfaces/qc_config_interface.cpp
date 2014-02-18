#include <ipcInterfaces/qc_config_interface.h>


int qc_config_subscribe_request_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_config_request_message..." << std::flush;
  if (IPC_subscribe(QC_CONFIG_REQUEST_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_CONFIG_REQUEST_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_config_publish_request_message (qc_config_request_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_CONFIG_REQUEST_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_config_request_message..." << std::flush;
    if (IPC_defineMsg(QC_CONFIG_REQUEST_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_CONFIG_REQUEST_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_CONFIG_REQUEST_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_config_request_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_config_unsubscribe_request_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_config_request_message..." << std::flush;
  if (IPC_unsubscribe(QC_CONFIG_REQUEST_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


///********* NEXT MESSAGE ***********///  


int qc_config_subscribe_parameter_message (HANDLER_TYPE handler, int queueLength, void* clientData) { 
  std::cerr << "Subscribing to qc_config_parameter_message..." << std::flush;
  if (IPC_subscribe(QC_CONFIG_PARAMETER_MESSAGE_NAME, handler, clientData) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    IPC_setMsgQueueLength((char*)QC_CONFIG_PARAMETER_MESSAGE_NAME, queueLength);
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


int qc_config_publish_parameter_message (qc_config_parameter_message* msg){ 
  if (msg == NULL){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED - message is NULL! " << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  if (!IPC_isMsgDefined(QC_CONFIG_PARAMETER_MESSAGE_NAME)){ 
    std::cerr << "Defining qc_config_parameter_message..." << std::flush;
    if (IPC_defineMsg(QC_CONFIG_PARAMETER_MESSAGE_NAME, IPC_VARIABLE_LENGTH, QC_CONFIG_PARAMETER_MESSAGE_FMT) == IPC_OK){ 
      std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
    } else { 
      std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
      return 0;
    }
  }
  if (IPC_publishData(QC_CONFIG_PARAMETER_MESSAGE_NAME, msg) != IPC_OK){ 
    std::cerr << ESC << COLOR_FAILED << "FAILED to publish qc_config_parameter_message!!" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
  return 1;
}


int qc_config_unsubscribe_parameter_message (HANDLER_TYPE handler) { 
  std::cerr << "Unsubscribing qc_config_parameter_message..." << std::flush;
  if (IPC_unsubscribe(QC_CONFIG_PARAMETER_MESSAGE_NAME, handler) == IPC_OK) { 
    std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl; 
    return 1;
  }
  else {
    std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl; 
    return 0;
  }
}


