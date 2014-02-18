#include "linefilereader.h"


LineFileReader::LineFileReader(){
	lis = 0;
}

LineFileReader::~LineFileReader(){
	if (lis)
		delete lis;
	lis = 0;
}

bool LineFileReader::openFile(const char* filename){
	in.open(filename);
	if (!in){
		std::cerr << "Error: unable to open file " << filename << std::endl;
		return false;
	}
	return true;
}

std::istringstream* LineFileReader::getNextLine(){
	if (in.good()){
		if (lis)
			delete lis;
		lis = 0;
		in.getline(buffer,LINE_FILE_READER_BUFFER_SIZE);
		lis = new std::istringstream(buffer);
		return lis;
	} else {
		return 0;
	}
	
}

bool LineFileReader::isGood(){
	return in.good();
}

void LineFileReader::close(){
	in.close();
}



