#ifndef _IPC_PARAM_READER_H_
#define _IPC_PARAM_READER_H_

#include <cstring>
#include <vector>
#include <ipc/ipc.h>
#include <ipcMessages/qc_config_messages.h>
#include <ipcInterfaces/qc_config_interface.h>
#include <cstdlib>
#include <cstdarg>
#include <cstdio>

#define TIMEOUT 100
//in ms
using std::string;
using std::vector;

class IPCParamReader{
	public:
		IPCParamReader();
		IPCParamReader(int _id);
		int readParam(const string& param, string& result);
		vector<int> readParams(const vector<string>& params, vector<string>& results);
		vector<string> toTokens(const string& result);
		
		
		///use this functions, they are cool :)
		///use them as follows
		///e.g. getIntValue (someintwith3Values, 3, &int1, &int2, &int3);
		///but BEWARE: no type check is performed! if you invoke getIntValue but give double as parameters, result will be crap!!
		int getIntValue(const string name, int numVars, ...);
		int getDoubleValue(const string name, int numVars, ...);
		int getFloatValue(const string name, int numVars, ...);
		int getStringValue(const string name, int numVars, ...);
		int getBoolValue(const string name, int numVars, ...);
		
	private:
		static void qc_config_parameter_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData);
		bool waitForMessage;
		int timeout;
		int id;
		qc_config_parameter_message pmsg;
		vector<string> names;
		vector<string> results;
		vector<int> valid;
};
#endif // _IPC_PARAM_READER_H_
