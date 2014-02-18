#include "ipcParamReader.h"

IPCParamReader::IPCParamReader(){
	id = (int) ((rand()/(double)RAND_MAX) * 1e10);
}

IPCParamReader::IPCParamReader(int _id){
	id = _id;
}
		
int IPCParamReader::readParam(const string& param, string& _result){
	waitForMessage = true;
	timeout = 0;
	qc_config_request_message rmsg;
	rmsg.numRequests = 1;
	rmsg.name = new char*[1];
	rmsg.name[0] = (char*)param.c_str();
	rmsg.requestID = id;
	qc_config_subscribe_parameter_message(IPCParamReader::qc_config_parameter_message_handler, 1, this);
	qc_config_publish_request_message(&rmsg);
	while (waitForMessage){
		IPC_listen(1);
		timeout++;
		if (timeout >= TIMEOUT)
			break;
	}
	qc_config_unsubscribe_parameter_message(IPCParamReader::qc_config_parameter_message_handler);
	if (waitForMessage)
		return 0;
	delete[] rmsg.name;
	if (valid.at(0) == 1)
		_result = results.at(0);
	return valid.at(0);
}

vector<int> IPCParamReader::readParams(const vector<string>& params, vector<string>& _results){
	waitForMessage = true;
	timeout = 0;
	qc_config_request_message rmsg;
	rmsg.numRequests = (int)params.size();
	rmsg.name = new char*[rmsg.numRequests];
	for (uint i=0; i<params.size(); i++)
		rmsg.name[i] = (char*)(params.at(i).c_str());
	rmsg.requestID = id;
	qc_config_subscribe_parameter_message(IPCParamReader::qc_config_parameter_message_handler, 1, this);
	qc_config_publish_request_message(&rmsg);
	while (waitForMessage){
		IPC_listen(1);
		timeout++;
		if (timeout >= TIMEOUT)
			break;
	}
	qc_config_unsubscribe_parameter_message(IPCParamReader::qc_config_parameter_message_handler);
	if (waitForMessage)
		return vector<int>(params.size(),0);
	delete[] rmsg.name;
	_results = results;
	return valid;
}

vector<string> IPCParamReader::toTokens(const string& s){
	vector<string> resultVector;
	char c[s.length()+1];
	for (uint i=0; i<s.length(); i++)
		c[i] = s[i];
	c[s.length()] = '\0';
	char delims[] = " ";
	char* result = NULL;
	result = strtok(c, delims);
	while (result != NULL){
		resultVector.push_back(string(result));
		result = strtok(NULL, delims);
	}
	return resultVector;
}


void IPCParamReader::qc_config_parameter_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	IPCParamReader* that = (IPCParamReader*) clientData;
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &that->pmsg, sizeof(that->pmsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	if (that->pmsg.requestID == that->id && that->waitForMessage){
		that->waitForMessage = false;
		that->names.resize(that->pmsg.numData);
		that->results.resize(that->pmsg.numData);
		that->valid.resize(that->pmsg.numData);
		for (int i=0; i<that->pmsg.numData; i++){
			that->valid[i] = that->pmsg.valid[i];
			that->names[i] = that->pmsg.name[i];
			if (that->pmsg.valid[i]){
				that->results[i] = that->pmsg.values[i];
			} 
		}
	}
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&that->pmsg);
}


int IPCParamReader::getIntValue(const string name, int numVars, ...){
	if (numVars == 0){
			std::cerr << "* Warning: numVars == 0!!" << std::endl;
			return 0;
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<int*> vars;
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,int*));
	va_end(ap);
	string resultParam;
	vector<string> tokens;
	if (readParam(name,resultParam)){
		tokens = toTokens(resultParam);
		if (tokens.size() != (uint)numVars){
			std::cerr << "Error: assuming " << numVars << " entries for \"" << name << "\" but i only found " << tokens.size() << std::endl;
			return 0;
		} 
	} else {
		std::cerr << "ERROR: parameter \"" << name << "\" not found! is configSender running? " << std::endl;
		return 0;
	}
	for (uint i=0; i<tokens.size(); i++)
		*vars.at(i) = atoi(tokens.at(i).c_str());
	return 1;
}

int IPCParamReader::getDoubleValue(const string name, int numVars, ...){
	if (numVars == 0){
			std::cerr << "* Warning: numVars == 0!!" << std::endl;
			return 0;
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<double*> vars;
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,double*));
	va_end(ap);
	string resultParam;
	vector<string> tokens;
	if (readParam(name,resultParam)){
		tokens = toTokens(resultParam);
		if (tokens.size() != (uint)numVars){
			std::cerr << "Error: assuming " << numVars << " entries for \"" << name << "\" but i only found " << tokens.size() << std::endl;
			return 0;
		} 
	} else {
		std::cerr << "ERROR: parameter \"" << name << "\" not found! is configSender running? " << std::endl;
		return 0;
	}
	for (uint i=0; i<tokens.size(); i++)
		*vars.at(i) = atof(tokens.at(i).c_str());
	return 1;
}

int IPCParamReader::getFloatValue(const string name, int numVars, ...){
	if (numVars == 0){
			std::cerr << "* Warning: numVars == 0!!" << std::endl;
			return 0;
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<float*> vars;
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,float*));
	va_end(ap);
	string resultParam;
	vector<string> tokens;
	if (readParam(name,resultParam)){
		tokens = toTokens(resultParam);
		if (tokens.size() != (uint)numVars){
			std::cerr << "Error: assuming " << numVars << " entries for \"" << name << "\" but i only found " << tokens.size() << std::endl;
			return 0;
		} 
	} else {
		std::cerr << "ERROR: parameter \"" << name << "\" not found! is configSender running? " << std::endl;
		return 0;
	}
	for (uint i=0; i<tokens.size(); i++)
		*vars.at(i) = (float)atof(tokens.at(i).c_str());
	return 1;
}

int IPCParamReader::getStringValue(const string name, int numVars, ...){
	if (numVars == 0){
			std::cerr << "* Warning: numVars == 0!!" << std::endl;
			return 0;
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<string*> vars;
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,string*));
	va_end(ap);
	string resultParam;
	vector<string> tokens;
	if (readParam(name,resultParam)){
		tokens = toTokens(resultParam);
		if (tokens.size() != (uint)numVars){
			std::cerr << "Error: assuming " << numVars << " entries for \"" << name << "\" but i only found " << tokens.size() << std::endl;
			return 0;
		} 
	} else {
		std::cerr << "ERROR: parameter \"" << name << "\" not found! is configSender running? " << std::endl;
		return 0;
	}
	for (uint i=0; i<tokens.size(); i++)
		*vars.at(i) = tokens.at(i);
	return 1;
}

int IPCParamReader::getBoolValue(const string name, int numVars, ...){
	if (numVars == 0){
			std::cerr << "* Warning: numVars == 0!!" << std::endl;
			return 0;
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<bool*> vars;
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,bool*));
	va_end(ap);
	string resultParam;
	vector<string> tokens;
	if (readParam(name,resultParam)){
		tokens = toTokens(resultParam);
		if (tokens.size() != (uint)numVars){
			std::cerr << "Error: assuming " << numVars << " entries for \"" << name << "\" but i only found " << tokens.size() << std::endl;
			return 0;
		} 
	} else {
		std::cerr << "ERROR: parameter \"" << name << "\" not found! is configSender running? " << std::endl;
		return 0;
	}
	
	for (uint i=0; i<tokens.size(); i++){
		char* c = (char*) tokens.at(i).c_str();
		int j=0;
		std::string tmp="";
		while (c[j])
			tmp = tmp + (char)tolower(c[j++]);
		if (tokens.at(i) == std::string("1") || tmp == std::string("true"))
			*vars.at(i) = true;
		else
			*vars.at(i) = false;
	}
	return 1;
}
