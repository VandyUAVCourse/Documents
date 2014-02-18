#include <ipcMessages/qc_config_messages.h>
#include <ipcInterfaces/qc_config_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <iostream>
#include <string>
#include <vector>
#include "ipcParamReader.h"

using namespace std;

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
					vector<string> requestNames(numReq);
					vector<char> requestTypes(numReq);
					bool _break = false;
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
						std::cerr << "Please enter type (d = double, f=float, s=string, b=bool, i=integer)" << std::endl;
						cin.getline(buffer,100);
						if (buffer[0] == 'c'){
							state = 0;
							_break = true;
							break;
						}
						if (buffer[0] == 'd' || buffer[0] == 'f' || buffer[0] == 's' || buffer[0] == 'i' || buffer[0] == 'b'){
							requestTypes[i] = buffer[0];
						}else{
							std::cerr << "Error unknown type, cancelling!" << std::endl;
							state = 0;
							_break = true;
							break;
						}
					}
					if (_break)
						break;
					IPCParamReader ipcParamReader;
					if (numReq == 1){
						string result;
						int valid = ipcParamReader.readParam(requestNames[0], result);
						if (valid){
							std::cerr << "FOUND " << requestNames[0] << " values=  "<< result << std::endl;
							std::cerr << "Tokens are " << std::endl;
							vector<string> tokens = ipcParamReader.toTokens(result);
							for (uint l=0; l<tokens.size(); l++){
								std::cerr << "l= "<< l << " tokens[l]= " << tokens[l] << std::endl;
							}
							std::cerr << "trying typecast reading" << std::endl;
							if (tokens.size() > 2)
								std::cerr << "I am stupid, I can only handle up to 2 values! " << std::endl;
							else{
								double d1,d2;
								float f1, f2;
								int i1, i2;
								string s1, s2;
								bool b1, b2;
								int valid = 0;
								if (requestTypes[0] == 'd'){
									if (tokens.size() == 1){
										valid = ipcParamReader.getDoubleValue(requestNames[0], 1, &d1);
										std::cerr << "valid= " << valid << " value= " << d1 << std::endl;
									}
									if (tokens.size() == 2){
										valid = ipcParamReader.getDoubleValue(requestNames[0], 2, &d1, &d2);
										std::cerr << "valid= " << valid << " values= " << d1 << " " << d2 <<  std::endl;
									}	
								}
								
								if (requestTypes[0] == 'f'){
									if (tokens.size() == 1){
										valid = ipcParamReader.getFloatValue(requestNames[0], 1, &f1);
										std::cerr << "valid= " << valid << " value= " << f1 << std::endl;
									}
									if (tokens.size() == 2){
										valid = ipcParamReader.getFloatValue(requestNames[0], 2, &f1, &f2);
										std::cerr << "valid= " << valid << " values= " << f1 << " " << f2 <<  std::endl;
									}	
								}
								
								if (requestTypes[0] == 'i'){
									if (tokens.size() == 1){
										valid = ipcParamReader.getIntValue(requestNames[0], 1, &i1);
										std::cerr << "valid= " << valid << " value= " << i1 << std::endl;
									}
									if (tokens.size() == 2){
										valid = ipcParamReader.getIntValue(requestNames[0], 2, &i1, &i2);
										std::cerr << "valid= " << valid << " values= " << i1 << " " << i2 <<  std::endl;
									}	
								}
								
								if (requestTypes[0] == 'b'){
									if (tokens.size() == 1){
										valid = ipcParamReader.getBoolValue(requestNames[0], 1, &b1);
										std::cerr << "valid= " << valid << " value= " << b1 << std::endl;
									}
									if (tokens.size() == 2){
										valid = ipcParamReader.getBoolValue(requestNames[0], 2, &b1, &b2);
										std::cerr << "valid= " << valid << " values= " << b1 << " " << b2 <<  std::endl;
									}	
								}
								
								if (requestTypes[0] == 's'){
									if (tokens.size() == 1){
										valid = ipcParamReader.getStringValue(requestNames[0], 1, &s1);
										std::cerr << "valid= " << valid << " value= " << s1 << std::endl;
									}
									if (tokens.size() == 2){
										valid = ipcParamReader.getStringValue(requestNames[0], 2, &s1, &s2);
										std::cerr << "valid= " << valid << " values= " << s1 << " " << s2 <<  std::endl;
									}	
								}
								
							}
						} else {
							std::cerr << "NOT FOUND " << requestNames[0] << std::endl;
						}
						

					} else {
						vector<string> results;
						vector<int> valid = ipcParamReader.readParams(requestNames, results);
						for (uint k=0; k<valid.size(); k++){
							if (valid[k]){
								std::cerr << "FOUND " << requestNames[k] << " values=  "<< results[k] << std::endl;
								std::cerr << "Tokens are " << std::endl;
								vector<string> tokens = ipcParamReader.toTokens(results[k]);
								for (uint l=0; l<tokens.size(); l++){
									std::cerr << "l= "<< l << " tokens[l]= " << tokens[l] << std::endl;
								}
							} else {
								std::cerr << "NOT FOUND " << requestNames[k] << std::endl;
							}
						}
					}
					state = 0;
					break;
		}
		
	}
	
	
	return 0;
}
