#include <ipc/ipc.h>
#include <ipcMessages/qc_config_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_config_interface.h>
#include <utils_global/cmdargsreader.h>
#include <utils_global/cfgreader.h>
#include <string>
#include <vector>
#include <cstdlib>

using std::string;
using std::vector;

CfgReader cfg;
qc_config_request_message rmsg;
qc_config_parameter_message pmsg;
string group = "";

void qc_config_request_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &rmsg, sizeof(rmsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	///get the message name, get cfg
	if (pmsg.name != 0){
		delete[] pmsg.name;
		pmsg.name = 0;
		delete[] pmsg.values;
		pmsg.values = 0;
		delete[] pmsg.valid;
		pmsg.valid = 0;
	}
	pmsg.numData = rmsg.numRequests;
	pmsg.requestID = rmsg.requestID;
	pmsg.name = new char*[pmsg.numData];
	pmsg.values = new char*[pmsg.numData];
	pmsg.valid = new int[pmsg.numData];
	vector<string> values(pmsg.numData);
	CfgReaderReturnValue returnValue;
	for (int i=0; i<rmsg.numRequests; i++){
		pmsg.name[i] = rmsg.name[i];
		string r(rmsg.name[i]);
		vector<string> cv = cfg.getStringValues(group,r, returnValue);
		if (cv.size() == 0){
			pmsg.values[i] = 0;
			pmsg.valid[i] = 0;
			continue;
		}
		pmsg.valid[i] = 1;
		string ct = "";
		for (uint j=0; j<cv.size(); j++){
			ct = ct + cv[j] + " ";
		}
		values[i] = ct;
		pmsg.values[i] = (char*)values[i].c_str();
	}
	if (!qc_config_publish_parameter_message(&pmsg))
		exit(-1);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&rmsg);
}




int main (int argc, char* argv[]){
	CmdArgsReader commandline(argc, argv);
	string cfgfilename = "";
	commandline.getStringValue("-f","-filename",&cfgfilename);
	commandline.getStringValue("-g","-group",&group);
	if (commandline.printUnusedArguments())
		return 0;
	if (argc < 5){
		std::cerr << "Usage= "<< argv[0] << " -f <filename> -g <groupname without \"[]\"> " << std::endl;
		return 0;
	}
		
	if (qc_ipc_connect(argv[0]) <= 0)
		return 0;
	
	
	if (!cfg.openFile(cfgfilename)){
		std::cerr << "Error : unable to open file \"" << cfgfilename << "\" " << std::endl;
		return 0;
	}
	
	CfgReaderReturnValue returnValue = cfg.isGroupPresent(group);
	if (returnValue != CfgReader_OK){
		std::cerr << (char)27 << "[31;1m";
		std::cerr << "Error: group \"" << group << "\" not found! ";
		std::cerr << (char)27 << "[0m" << std::endl;
		return 0;
	}
	
	qc_config_subscribe_request_message(qc_config_request_message_handler, 10, NULL);
	
	IPC_dispatch();
	
	return 0;
}
