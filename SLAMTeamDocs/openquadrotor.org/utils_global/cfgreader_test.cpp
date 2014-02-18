#include "cfgreader.h"

#include <iostream>
#include <vector>

int main (int argc, char* argv[]){
  if (argc==1){
	std::cerr << "Usage: " << argv[0] << " -f <filename> " << std::endl;
	return -1;
  }
  
  std::string filename;
  int offset=1;
  while (offset < argc){
		if (std::string(argv[offset]) == std::string("-f")) {
			offset++;
			filename = argv[offset];
			offset++;
			continue;
		} else {
			std::cerr << "Unknown Command " << argv[offset] << std::endl;
			return -1;
		}
  }
	
  CfgReader cfgreader;
	cfgreader.openFile(filename);
		
	bool quit = false;
	std::string group = "";
	std::string name = "";
	int value =0;
	while (!quit){
		std::cerr << "Enter Group (enter \"\" for \"none\"): " << std::flush;
		group = "";
		name = "";
		value = -1;
		std::cin >> group;
		if (group == "\"\"")
			group = "";
		std::cerr << "Enter Keyword: " << std::flush;
		std::cin >> name;
		while (value < 0 || value > 3){
			std::cerr << "Enter Type [0=int, 1=double, 2=bool, 3=string]: ";
			std::cin >> value;
		}
		std::vector<int> vi;
		std::vector<double> vd;
		std::vector<bool> vb;
		std::vector<std::string> vs;
		CfgReaderReturnValue returnValue;
		switch (value){
			default:
				break;
			case 0:
				vi = cfgreader.getIntValues(group,name,returnValue);
				std::cerr << std::endl << "Found Values= " << vi.size() << std::endl;
				for (uint i=0; i<vi.size(); i++)
					std::cerr << vi.at(i) << " ";
				std::cerr << std::endl << std::endl;
				break;
			case 1:
				vd = cfgreader.getDoubleValues(group,name,returnValue);
				std::cerr <<std::endl << "Found Values= " << vd.size() << std::endl;
				for (uint i=0; i<vd.size(); i++)
					std::cerr << vd.at(i) << " ";
				std::cerr << std::endl << std::endl;
				break;
			case 2:
				vb = cfgreader.getBoolValues(group,name,returnValue);
				std::cerr <<std::endl << "Found Values= " << vb.size() << std::endl;
				for (uint i=0; i<vb.size(); i++)
					std::cerr << vb.at(i) << " ";
				std::cerr << std::endl << std::endl;
				break;
			case 3:
				vs = cfgreader.getStringValues(group,name,returnValue);
				std::cerr <<std::endl << "Found Values= " << vs.size() << std::endl;
				for (uint i=0; i<vs.size(); i++)
					std::cerr << vs.at(i) << " ";
				std::cerr << std::endl << std::endl;
				break;
		}
		std::cerr << "Quit Program (1) or query again (0)? " << std::flush;
		std::cin >> quit;
	}
	  return 0;
}
