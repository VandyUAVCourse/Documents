#include "cmdargsreader.h"

CmdArgsReader::CmdArgsReader(int argc, char* argv[]){
	for (int i=1; i<argc; i++){
		values.push_back(argv[i]);
		usedValues.push_back(false);
	}
	displayWarnings = true;
	displayFoundVars = true;
}

void CmdArgsReader::set(bool _displayWarnings, bool _displayFoundVars){
	displayWarnings = _displayWarnings;
	displayFoundVars = _displayFoundVars;
}

int CmdArgsReader::printUnusedArguments(){
	int unused=0;
	for (uint i=0; i < usedValues.size(); i++)
		if (!usedValues.at(i))
			unused++;
	if (unused == 0)
		return 0;
	
	if (displayWarnings){
		begin_warning();
		std::cerr << "* Warning: Unknown or Unused Parameters: " << std::endl;
		for (uint i=0; i<usedValues.size(); i++){
			if (!usedValues.at(i)){
				std::cerr << values.at(i) << " " << std::flush;
			}
		}
		std::cerr << std::endl;
		end_warning();
	}
	return 1;
}


///INT VALUE


int CmdArgsReader::getIntValue(const char* name, int* var){
	return getIntValue(1,1,name,var);
}

int CmdArgsReader::getIntValue(const char* name1, const char* name2, int* var){
	return getIntValue(2,1,name1,name2,var);
}

int CmdArgsReader::getIntValue(int numNames, int numVars, ...){
	if (numNames == 0 || numVars == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<std::string> names;
	std::vector<int*> vars;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap,char*));
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,int*));
	va_end(ap);
	
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0 && returnVector.size() != (uint) numVars){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: Assuming for " << names.at(0) << " " << numVars << " entries.";
			std::cerr << "But just found " << returnVector.size() << " entries! Aborting! " << std::endl;
			end_warning();
		}
		return 0;
	}
	for (uint i=0; i<returnVector.size(); i++)
		*vars.at(i) = atoi(returnVector.at(i).c_str());
	return 1;
}

///DOUBLE VALUE

int CmdArgsReader::getDoubleValue(const char* name, double* var){
	return getDoubleValue(1,1,name,var);
}

int CmdArgsReader::getDoubleValue(const char* name1, const char* name2, double* var){
	return getDoubleValue(2,1,name1,name2,var);
}

int CmdArgsReader::getDoubleValue(int numNames, int numVars, ...){
	if (numNames == 0 || numVars == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<std::string> names;
	std::vector<double*> vars;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap,char*));
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,double*));
	va_end(ap);
	
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0 && returnVector.size() != (uint) numVars){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: Assuming for " << names.at(0) << " " << numVars << " entries.";
			std::cerr << "But just found " << returnVector.size() << " entries! Aborting! " << std::endl;
			end_warning();
		}
		return 0;
	}
	for (uint i=0; i<returnVector.size(); i++)
		*vars.at(i) = atof(returnVector.at(i).c_str());
	return 1;
}

///FLOAT VALUE

int CmdArgsReader::getFloatValue(const char* name, float* var){
	return getFloatValue(1,1,name,var);
}

int CmdArgsReader::getFloatValue(const char* name1, const char* name2, float* var){
	return getFloatValue(2,1,name1,name2,var);
}

int CmdArgsReader::getFloatValue(int numNames, int numVars, ...){
	if (numNames == 0 || numVars == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<std::string> names;
	std::vector<float*> vars;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap,char*));
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,float*));
	va_end(ap);
	
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0 && returnVector.size() != (uint) numVars){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: Assuming for " << names.at(0) << " " << numVars << " entries.";
			std::cerr << "But just found " << returnVector.size() << " entries! Aborting! " << std::endl;
			end_warning();
		}
		return 0;
	}
	for (uint i=0; i<returnVector.size(); i++)
		*vars.at(i) = (float) atof(returnVector.at(i).c_str());
	return 1;
}



///STRING VALUE

int CmdArgsReader::getStringValue(const char* name, std::string* var){
	return getStringValue(1,1,name,var);
}

int CmdArgsReader::getStringValue(const char* name1, const char* name2, std::string* var){
	return getStringValue(2,1,name1,name2,var);
}

int CmdArgsReader::getStringValue(int numNames, int numVars, ...){
	if (numNames == 0 || numVars == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<std::string> names;
	std::vector<std::string*> vars;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap, char*));
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap, std::string*));
	va_end(ap);
	
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0 && returnVector.size() != (uint) numVars){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: Assuming for " << names.at(0) << " " << numVars << " entries.";
			std::cerr << "But just found " << returnVector.size() << " entries! Aborting! " << std::endl;
			end_warning();
		}
		return 0;
	}
	
	for (uint i=0; i<returnVector.size(); i++){
		*vars.at(i) = returnVector.at(i);
	}
	return 1;
}




///BOOL VALUE

int CmdArgsReader::getBoolValue(const char* name, bool* var){
	return getBoolValue(1,1,name,var);
}

int CmdArgsReader::getBoolValue(const char* name1, const char* name2, bool* var){
	return getBoolValue(2,1,name1,name2,var);
}

int CmdArgsReader::getBoolValue(int numNames, int numVars, ...){
	if (numNames == 0 || numVars == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, numVars);
	std::vector<std::string> names;
	std::vector<bool*> vars;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap,char*));
	for (int i=0; i<numVars; i++)
		vars.push_back(va_arg(ap,bool*));
	va_end(ap);
	
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0 && returnVector.size() != (uint) numVars){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: Assuming for " << names.at(0) << " " << numVars << " entries.";
			std::cerr << "But just found " << returnVector.size() << " entries! Aborting! " << std::endl;
			end_warning();
		}
		return 0;
	}
	for (uint i=0; i<returnVector.size(); i++){
		char* c = (char*) returnVector.at(i).c_str();
		int j=0;
		std::string tmp="";
		while (c[j])
			tmp = tmp + (char)tolower(c[j++]);
		if (returnVector.at(i) == std::string("1") || tmp == std::string("true"))
			*vars.at(i) = true;
		else
			*vars.at(i) = false;
	}
		
	return 1;
}


///FLAG VALUE

int CmdArgsReader::getFlagValue(const char* name, bool* var){
	return getFlagValue(1,var,name);
}

int CmdArgsReader::getFlagValue(const char* name1, const char* name2, bool* var){
	return getFlagValue(2,var,name1,name2);
}

int CmdArgsReader::getFlagValue(int numNames, bool* var, ...){
	if (numNames == 0){
		if (displayWarnings){
			begin_warning();
			std::cerr << "* Warning: numNames == 0 or numVars == 0!!" << std::endl;
			end_warning();
			return 0;
		}
	}
	va_list ap;
	va_start(ap, var);
	std::vector<std::string> names;
	for (int i=0; i<numNames; i++)
		names.push_back(va_arg(ap,char*));
	va_end(ap);
	int numVars = 0;
	std::vector<std::string> returnVector = getValues(names, numVars);
	if (returnVector.size() > 0)
		*var = true;
	else
		*var = false;
	return 1;
}

//############################# protected ######################################

std::vector<std::string> CmdArgsReader::getValues(std::vector<std::string>& names, int& numValues){
	bool foundString = false;
	int readValues=0;
	std::vector<std::string> returnVector;
	for (uint i=0; i<values.size(); i++){
		if (foundString){
			if (readValues < numValues){
				usedValues[i] = true;
				returnVector.push_back(values.at(i));
				readValues++;
				if (displayFoundVars){
					begin_info();
					std::cerr << values.at(i) << " " << std::flush;
					end_info();
					if (i==values.size() - 1)
						std::cerr << std::endl;
				}
			} else{
				if (displayFoundVars)
					std::cerr << std::endl;
				break;
			}
				
		}
		for (uint j=0; j<names.size(); j++){
			if (values.at(i) == names.at(j)){
				foundString = true;
				usedValues[i] = true;
				if (displayFoundVars){
					begin_info();
					std::cerr << "* Found: " << values.at(i) << " " << std::flush;
					end_info();
					if (i == (values.size() - 1))
						std::cerr << std::endl;
				}
				if (numValues == 0)
					returnVector.push_back("found");
				break;
			}
		}
	}
	return returnVector;
}

void CmdArgsReader::begin_warning(){
	std::cerr << (char)27 << "[31;1m" << std::flush;
}

void CmdArgsReader::end_warning(){
	std::cerr << (char)27 << "[0m" << std::flush;
}

void CmdArgsReader::begin_info(){
	std::cerr << (char)27 << "[34;1m" << std::flush;
}

void CmdArgsReader::end_info(){
	std::cerr << (char)27 << "[0m" << std::flush;
}

