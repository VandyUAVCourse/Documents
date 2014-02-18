///this program will generate the code for one messgages file
///the program is searching for #defines XXX_NAME and XXX_FMT
///and then generates a header and a cpp file in the desired folder.

#include "messageStruct.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>

using namespace std;
int state = 0;
string inFilename = "";
string outDirectory = "";
string outFilenamePrefix = "";
string inFileRaw = "";

bool checkCorrectName(string s){
	if (s.substr(s.length()-4) == "NAME")
		return true;
	return false;
}

bool checkCorrectFMT(string pre, string s){
	string s1 = pre.substr(0,pre.length()-4);
	string s2 = s.substr(0,s.length()-3);
	if (s1 == s2)
		return true;
	return false;
}

bool checkCorrectTYPE(string pre, string s){
	string s1 = pre.substr(0,pre.length()-4);
	string s2 = s.substr(0,s.length()-4);
	if (s1 == s2)
		return true;
	return false;
}

int main (int argc, char* argv[]){
	if (argc != 4){
		std::cerr << "**Usage= " << argv[0] << " <infile> <outDir> <outFilenamePrefix without .h or .cpp> " << std::endl;
		std::cerr << "**Example: " << argv[0] << " ./ipcMessages/qc_laser_messages.h ./ipcInterfaces/ qc_laser_interface" << std::endl;
		return 0;
		
	}
	inFilename = argv[1];
	outDirectory = argv[2];
	outFilenamePrefix = argv[3];
	int iii = inFilename.rfind('/');
	inFileRaw = inFilename.substr(iii+1);
		
	ifstream in(inFilename.c_str());
	if (!in){
		std::cerr << "Error: unable to open file: " << inFilename << std::endl;
		return 0;
	}
	if (outDirectory.at(outDirectory.length()-1) != '/')
		outDirectory = outDirectory + "/";
	string outFilenameH = outDirectory + outFilenamePrefix + ".h";
	string outFilenameCPP = outDirectory + outFilenamePrefix + ".cpp";
	std::cerr << "===> Will create files " << outFilenameH << " and " << outFilenameCPP << std::endl;
	std::cerr << "######## BEWARE: i assume define messages in the following order XXX_NAME then XXX_FMT then XXX_TYPE" << std::endl;
	std::cerr << "######## Remember, I am a silly program!" << std::endl;
	
	int state = 0;
	vector<MessageEntry> messages;
	MessageEntry m;
	while (in.good()){
		char buffer[8192];
		in.getline(buffer,8192);
		istringstream lis(buffer);
		string tag;
		lis >> tag;
		string pre, post;
		if (tag == "#define") {
			lis >> pre >> post;
			switch (state){
				case 0:
					if (checkCorrectName(pre)){
						m.pre_defined_name = pre;
						if (post.at(0) == '"')
							post = post.substr(1, post.length()-1);
						if (post.at(post.length()-1) == '"')
							post = post.substr(0, post.length()-1);
						m.post_defined_name = post;
						state ++;
					}
					break;
				case 1:
					if (checkCorrectFMT(m.pre_defined_name,pre)){
						m.pre_defined_fmt = pre;
						state++;
					} else {
						std::cerr << "Error: expected format of the previous defined name but found something else! " << std::endl;
						std::cerr << "name= " << m.pre_defined_name << std::endl;
						std::cerr<< "fmt= " << pre << std::endl;
						std::cerr << "resetting (therefore ommiting entry)" << std::endl;
						state = 0;
					}
					break;
				case 2:
					if (checkCorrectTYPE(m.pre_defined_name, pre)){
						m.post_defined_type = post;
					} else {
						std::cerr << "Warning: no type declaration found, will use IPC_VARIABLE_LENGTH" << std::endl;
						m.post_defined_type = "IPC_VARIABLE_LENGTH";
					}
					messages.push_back(m);
					m.pre_defined_name = "";
					m.pre_defined_fmt = "";
					m.post_defined_name = "";
					m.post_defined_type = "";
					state = 0;
					if (!checkCorrectTYPE(m.pre_defined_name, pre)){
						///could be a define of a new message
						///(== state 0 already)
						if (checkCorrectName(pre)){
							m.pre_defined_name = pre;
							if (post.at(0) == '"')
								post = post.substr(1, post.length()-1);
							if (post.at(post.length()-1) == '"')
								post = post.substr(0, post.length()-1);
							m.post_defined_name = post;
							state = 2;
						}
					}
						
					break;
				default:
					state = 0;
					break;
			}
		} else {
			if (state == 2){
				std::cerr << "Warning: no type declaration found, will use IPC_VARIABLE_LENGTH" << std::endl;
				m.post_defined_type = "IPC_VARIABLE_LENGTH";
				messages.push_back(m);
				m.pre_defined_name = "";
				m.pre_defined_fmt = "";
				m.post_defined_name = "";
				m.post_defined_type = "";
			}
			state = 0;
		}
	}
	std::cerr << "I found " << messages.size() << " different messages " << std::endl;
	for (uint i=0; i< messages.size(); i++){
		std::cerr << "Message " << i << std::endl;
		std::cerr << "== " << messages.at(i).pre_defined_name << std::endl;
		std::cerr << "== " << messages.at(i).post_defined_name << std::endl;
		std::cerr << "== " << messages.at(i).pre_defined_fmt << std::endl;
		std::cerr << "== " << messages.at(i).post_defined_type << std::endl;
		std::cerr << std::endl;
	}
	in.close();
	
	
	ofstream outH(outFilenameH.c_str());
	if (!outH){
		std::cerr << "Error: unable to open file " << outFilenameH << std::endl;
		return 0;
	}
	ofstream outCPP(outFilenameCPP.c_str());
	if (!outCPP){
		std::cerr << "Error: unable to open file " << outFilenameCPP << std::endl;
		outH.close();
		return 0;
	}
	
	string includestring = outFilenamePrefix;
	for (int j=0; j<(int)outFilenamePrefix.length(); j++)
		outFilenamePrefix[j] = toupper(outFilenamePrefix[j]);
	
	for (uint i=0;  i<messages.size(); i++){
		int idx1 = messages.at(i).post_defined_name.find('_',0);
		int idx2 = messages.at(i).post_defined_name.find('_',idx1+1);
		if (idx2 == (int)string::npos ){
			std::cerr << "DID NOT FIND?! " << idx1 << " " << idx2 << std::endl;
			continue;
		}
		string name1 = messages.at(i).post_defined_name.substr(0,idx2+1);
		string name2 = messages.at(i).post_defined_name.substr(idx2+1, messages.at(i).post_defined_name.length()-1);
		
		///### H FILE
		
		if (i == 0){
			outH << "#ifndef _"<< outFilenamePrefix << "_H_" << std::endl;
			outH << "#define _"<< outFilenamePrefix << "_H_" << std::endl;
			outH << std::endl ;
			outH << "#include <iostream> " << std::endl;
			outH << "#include <ipc/ipc.h> " << std::endl;
			outH << "#include <utils_ipc/colormakros.h> " << std::endl;
			outH << "#include <ipcMessages/" << inFileRaw << ">" << std::endl;
			outH << std::endl << std::endl;
		}
		
		outH << "int " << name1 << "subscribe_" << name2 << " (HANDLER_TYPE handler, int queueLength = 1, void* clientData = NULL); " << std::endl;
		
		
		outH << "int " << name1 << "publish_" << name2 << " (" << messages.at(i).post_defined_name << "* msg); " << std::endl;
		
		outH << "int " << name1 << "unsubscribe_" << name2 << " (HANDLER_TYPE handler); " << std::endl;
		
		outH << std::endl << std::endl << std::endl;
		
		if ( i < messages.size()-1)
			outH << "///********* NEXT MESSAGE ***********///  " << std::endl << std::endl << std::endl;
		
		if (i == messages.size() -1)
			outH << "#endif // _" << outFilenamePrefix << "_H_" << std::endl;
		
		///### CPP FILE
		if (i == 0){
			outCPP << "#include <ipcInterfaces/" << includestring << ".h>" << std::endl;
			outCPP << std::endl << std::endl;
		}
		outCPP << "int " << name1 << "subscribe_" << name2 << " (HANDLER_TYPE handler, int queueLength, void* clientData) { " << std::endl;
		outCPP << "  std::cerr << \"Subscribing to "<< messages.at(i).post_defined_name << "...\" << std::flush;" << std::endl;
		outCPP << "  if (IPC_subscribe("<<messages.at(i).pre_defined_name<<", handler, clientData) == IPC_OK) { " << std::endl;
			outCPP << "    std::cerr << ESC << COLOR_SUCCESS << \"SUCCESS\" << ESC << COLOR_RESET << std::endl; " << std::endl;
			outCPP << "    IPC_setMsgQueueLength((char*)"<<messages.at(i).pre_defined_name<<", queueLength);" << std::endl;
			outCPP << "    return 1;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "  else {" << std::endl;
		outCPP << "    std::cerr << ESC << COLOR_FAILED << \"FAILED\" << ESC << COLOR_RESET << std::endl; " << std::endl;
		outCPP << "    return 0;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "}"<< std::endl;
		outCPP << std::endl << std::endl;
		
		
		outCPP << "int " << name1 << "publish_" << name2 << " (" << messages.at(i).post_defined_name << "* msg){ " << std::endl;
		outCPP << "  if (msg == NULL){ " << std::endl;
		outCPP << "    std::cerr << ESC << COLOR_FAILED << \"FAILED - message is NULL! \" << ESC << COLOR_RESET << std::endl; " << std::endl;
		outCPP << "    return 0;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "  if (!IPC_isMsgDefined("<<messages.at(i).pre_defined_name<<")){ " << std::endl;
		outCPP << "    std::cerr << \"Defining " << messages.at(i).post_defined_name << "...\" << std::flush;" << std::endl;
		outCPP << "    if (IPC_defineMsg("<< messages.at(i).pre_defined_name <<", "<< messages.at(i).post_defined_type <<", "<< messages.at(i).pre_defined_fmt <<") == IPC_OK){ " << std::endl;
		outCPP << "      std::cerr << ESC << COLOR_SUCCESS << \"SUCCESS\" << ESC << COLOR_RESET << std::endl;" << std::endl;
	   outCPP << "    } else { " << std::endl;
		outCPP << "      std::cerr << ESC << COLOR_FAILED << \"FAILED\" << ESC << COLOR_RESET << std::endl;" << std::endl;
		outCPP << "      return 0;" << std::endl;
		outCPP << "    }" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "  if (IPC_publishData("<<messages.at(i).pre_defined_name<<", msg) != IPC_OK){ " << std::endl;
		outCPP << "    std::cerr << ESC << COLOR_FAILED << \"FAILED to publish "<< messages.at(i).post_defined_name << "!!\" << ESC << COLOR_RESET << std::endl; " << std::endl;
		outCPP << "    return 0;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "  return 1;" << std::endl;
	   outCPP << "}" << std::endl;
		outCPP << std::endl << std::endl;
		
		outCPP << "int " << name1 << "unsubscribe_" << name2 << " (HANDLER_TYPE handler) { " << std::endl;
		outCPP << "  std::cerr << \"Unsubscribing "<< messages.at(i).post_defined_name << "...\" << std::flush;" << std::endl;
		outCPP << "  if (IPC_unsubscribe("<<messages.at(i).pre_defined_name<<", handler) == IPC_OK) { " << std::endl;
		outCPP << "    std::cerr << ESC << COLOR_SUCCESS << \"SUCCESS\" << ESC << COLOR_RESET << std::endl; " << std::endl;
		outCPP << "    return 1;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "  else {" << std::endl;
		outCPP << "    std::cerr << ESC << COLOR_FAILED << \"FAILED\" << ESC << COLOR_RESET << std::endl; " << std::endl;
		outCPP << "    return 0;" << std::endl;
		outCPP << "  }" << std::endl;
		outCPP << "}"<< std::endl;
		outCPP << std::endl << std::endl;
		
		
		if ( i < messages.size()-1)
			outCPP << "///********* NEXT MESSAGE ***********///  " << std::endl << std::endl << std::endl;
	}
	
	outH.close();
	outCPP.close();
}
