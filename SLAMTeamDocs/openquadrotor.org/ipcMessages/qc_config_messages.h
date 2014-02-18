#ifndef _QC_CONFIG_MESSAGES_H_
#define _QC_CONFIG_MESSAGES_H_

struct qc_config_request_message {
	int numRequests;
	char** name;
	int requestID;
	qc_config_request_message(){
		numRequests = 0;
		name = 0;
		requestID = 0;
	}
};

#define QC_CONFIG_REQUEST_MESSAGE_NAME "qc_config_request_message" 
#define QC_CONFIG_REQUEST_MESSAGE_FMT "{int, <string:1>, int}"

struct qc_config_parameter_message{
	int numData;
	char** name;
	char** values;
	int* valid;
	int requestID;
	qc_config_parameter_message(){
		numData = 0;
		name = 0;
		values = 0;
		valid = 0;
		requestID = 0;
	}
};

#define QC_CONFIG_PARAMETER_MESSAGE_NAME "qc_config_parameter_message"
#define QC_CONFIG_PARAMETER_MESSAGE_FMT "{int, <string:1>, <string:1>, <int:1>, int}"

#endif // _QC_CONFIG_MESSAGES_H_
