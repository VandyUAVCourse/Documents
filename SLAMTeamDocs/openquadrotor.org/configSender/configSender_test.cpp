#include <ipcMessages/qc_config_messages.h>
#include <ipcInterfaces/qc_config_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using namespace std;

qc_config_request_message rmsg;
qc_config_parameter_message pmsg;
int globalRequestID = 100;
bool waitForMessage = true;

void qc_config_parameter_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &pmsg, sizeof(pmsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	std::cerr << "Received state message " << std::endl;
	std::cerr << "RequestId= "<< pmsg.requestID << std::endl;
	std::cerr << "NumData= " << pmsg.numData << std::endl;
	for (int i=0; i<pmsg.numData; i++){
		if (pmsg.valid[i]){
			std::cerr << "FOUND "<< pmsg.name[i] << " : " << pmsg.values[i] << std::endl;
		} else {
			std::cerr << "NOT FOUND "<< pmsg.name[i] << "!" << std::endl;
		}
		
	}
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&pmsg);
	waitForMessage = false;
}

int main (int argc, char* argv[]){
	bool stop = false;
	int state = 0;
	char buffer[100];
	if (qc_ipc_connect(argv[0]) <= 0)
		return 0;
	while (!stop){
		switch (state){
			case 0: ///main menu
				std::cerr << "choose q for quit or r for request" << std::endl;
				cin.getline(buffer,10);
				if (buffer[0] == 'q')
					stop = true;
				if (buffer[0] == 'r')
					state = 1;
				break;
			case 1: ///request menu
				std::cerr << "Request Menu: (always c for cancel) " << std::endl;
				std::cerr << "NumRequests(max 9): "<< std::flush;
				cin.getline(buffer,10);
				if (buffer[0] == 'c'){
					state = 0;
					break;
				}
				int numReq = atoi(buffer);
				if (numReq <= 0){
					std::cerr << "invalid number! " << std::endl;
					break;
				}
				rmsg.numRequests = numReq;
				vector<string> requestNames(numReq);
				bool _break = false;
				if (rmsg.name != 0)
					delete[] rmsg.name;
				rmsg.name = new char*[numReq];
				std::cerr << "You asked for " << numReq << " parameters " << std::endl;
				for (int i=0; i<numReq; i++){
					std::cerr << "Please enter name for parameter " << i << " : " << std::flush;
					cin.getline(buffer,100);
					requestNames[i] = buffer;
					if (requestNames[i] == "c"){
						state = 0;
						_break = true;
						break;
					}
					rmsg.name[i] = (char*) requestNames[i].c_str();
				}
				if (_break)
					break;
				rmsg.requestID = globalRequestID;
				qc_config_subscribe_parameter_message(qc_config_parameter_message_handler,10,NULL);
				waitForMessage = true;
				qc_config_publish_request_message(&rmsg);
				while (waitForMessage){
					IPC_listen(1);
				}
				qc_config_unsubscribe_parameter_message(qc_config_parameter_message_handler);
				state = 0;
				break;
		}
		
	}
	
	
	return 0;
}
