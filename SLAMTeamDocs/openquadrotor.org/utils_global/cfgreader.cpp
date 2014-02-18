#include "cfgreader.h"
#include <sstream>
#include <vector>

#define INPUT_BUFFER 16384

CfgReader::CfgReader(){
 displayOnError = true;
}

CfgReader::~CfgReader(){
	if (in.is_open())
		in.close();
}

void CfgReader::setDisplayOnError(bool _displayOnError){
	displayOnError = _displayOnError;
}

int CfgReader::openFile (const std::string filename){
	if (in.is_open())
		in.close();
	in.open(filename.c_str());
	if (!in)
		return 0;
	else
		return 1;
}

std::vector<int> CfgReader::getIntValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue){
	readValues(group,name, returnValue);
	std::vector<int> returnVector;
	for (uint i=0; i<values.size(); i++)
		returnVector.push_back(atoi(values.at(i).c_str()));
	return returnVector;
}

std::vector<double> CfgReader::getDoubleValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue){
	readValues(group,name, returnValue);
	std::vector<double> returnVector;
	for (uint i=0; i<values.size(); i++)
		returnVector.push_back(atof(values.at(i).c_str()));
	return returnVector;
}

std::vector<std::string> CfgReader::getStringValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue){
	readValues(group,name, returnValue);
	return values;
}

std::vector<bool> CfgReader::getBoolValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue){
	readValues(group,name, returnValue);
	std::vector<bool> returnVector;
	
	for (uint i=0; i<values.size(); i++){
		std::string tmp = "";
		const char* c = values.at(i).c_str();
		int j=0;
		while (c[j]){
			tmp = tmp + (char)tolower(c[j++]);
		}
		if (values.at(i) == std::string("1") || tmp == std::string("true"))
			returnVector.push_back(true);
		else
			returnVector.push_back(false);
	}
	return returnVector;
}

//########################## protected #########################################
void CfgReader::readValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue){
	std::string currentGroup="[]";
	std::string searchGroup="["+group+"]";
	bool foundGroup = false;
	bool foundGroupAtLeastOnce = false;
	bool foundEntry = false;
	char buffer[INPUT_BUFFER];
	in.clear();
	in.seekg(0,std::ios::beg);
	values.clear();
	if (currentGroup == searchGroup){
		foundGroup = true;
		foundGroupAtLeastOnce = true;
	}
	while (in.good()){
		in.getline(buffer,INPUT_BUFFER);
		std::stringstream lis(buffer);
		std::string entry;
		foundEntry = false;
		while (lis.good()){
			lis >> entry;
			if (entry.substr(0,1) == std::string("#")) //comment
				break;
			
			if (entry.substr(0,1) == std::string("[")){ //group tag
				currentGroup = entry;
				if (currentGroup == searchGroup){
					foundGroup = true;
					foundGroupAtLeastOnce = true;
				}
				else
					foundGroup = false;
				continue;
			}
			
			if (foundGroup && entry == name){ //entry tag
				foundEntry = true;
				continue;
			}
			
			if (foundGroup && foundEntry){ //value
				values.push_back(entry);
			}
			
		}
		if (foundEntry && foundGroup) //values found -> abort search
			break;
	}
	if (!foundGroupAtLeastOnce)
		returnValue = CfgReader_GROUP_NOT_FOUND;
	else if (!foundEntry)
		returnValue = CfgReader_VALUE_NOT_FOUND;
	else
		returnValue = CfgReader_OK;
	in.clear();
	in.seekg(0,std::ios::beg);
	
	if(displayOnError && (returnValue == CfgReader_GROUP_NOT_FOUND)){
		std::cerr << (char)27 << "[31;1m Error: Group \"" << group << "\" not found. Remember: a \"[\" \"]\" is placed by default around the group name";
		std::cerr << (char)27 << "[0m" << std::endl;
	}
	
	if(displayOnError && (returnValue == CfgReader_VALUE_NOT_FOUND)){
		std::cerr << (char)27 << "[31;1m Error: Value \"" << name << "\" not found. I searched in group \"" << group << "\"";
		std::cerr << (char)27 << "[0m" << std::endl;
	}
}

CfgReaderReturnValue CfgReader::isGroupPresent(const std::string group){
	std::string currentGroup="[]";
	std::string searchGroup="["+group+"]";
	bool foundGroup = false;
	char buffer[INPUT_BUFFER];
	if (currentGroup == searchGroup)
		foundGroup = true;
	else {
		while (in.good()){
			in.getline(buffer,INPUT_BUFFER);
			std::stringstream lis(buffer);
			std::string entry;
			while (lis.good()){
				lis >> entry;
				if (entry.substr(0,1) == std::string("#")) //comment
					break;
				if (entry.substr(0,1) == std::string("[")){ //group tag
					currentGroup = entry;
					if (currentGroup == searchGroup)
						foundGroup = true;
					else
						foundGroup = false;
					continue;
				}
			}
			if (foundGroup) //values found -> abort search
				break;
		}
	}
	in.clear();
	in.seekg(0,std::ios::beg);
	if (foundGroup)
		return CfgReader_OK;
	else 
		return CfgReader_GROUP_NOT_FOUND;
	
}


