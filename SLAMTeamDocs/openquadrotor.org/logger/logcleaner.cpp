#include <utils_global/cmdargsreader.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

int main (int argc, char* argv[]){
	if (argc < 2){
		std::cerr << "Usage= " << argv[0] << " -f <logfile> " << std::endl;
		return 0;
	}
	CmdArgsReader commandline(argc, argv);
	string filename = "";
	commandline.getStringValue("-f","-filename",&filename);
	if (commandline.printUnusedArguments())
		return 0;
	///clean up logs, this means only raw messages will be kept
	///raw messages are: jpg, imu, laser, flightcontrol
	ifstream in(filename.c_str());
	string outfilename = filename + ".cleaned";
	ofstream out (outfilename.c_str());
	
	if (!in){
		std::cerr << "Error, unable to open file " << filename << std::endl;
		return 0;
	}
	
	if (!out){
		std::cerr << "Error, unable to open file " << outfilename << std::endl;
		return 0;
	}
	while (in.good()){
		char buffer[100000];
		in.getline(buffer, 100000);
		istringstream lis(buffer);
		string tag;
		lis >> tag;
		if (buffer[0] == '#' || tag == "IMU" || tag == "LASER1" || tag == "LASER2" || tag == "WEBCAM_JPG_PICTURE" || tag == "FLIGHTCONTROL" || tag == "OFFBOARDCOMMAND"){
			out << buffer << std::endl;
			std::cerr << "*" << std::flush;
		}
		std::cerr << "." << std::flush;
	}
	std::cerr << std::endl << "FINISHED " << std::endl;
	in.close();
	out.close();
	return 0;
}
