#include <iostream>
#include <fstream>
#include <sstream>

#ifndef _LINE_FILE_READER_H_
#define _LINE_FILE_READER_H_

#define LINE_FILE_READER_BUFFER_SIZE 50000

class LineFileReader{
	public:
		LineFileReader();
		~LineFileReader();
		bool openFile(const char* filename);
		std::istringstream* getNextLine();
		bool isGood();
		void close();


	private:
		char buffer[LINE_FILE_READER_BUFFER_SIZE];
		std::istringstream* lis;
		std::ifstream in;


};

#endif// _LINE_FILE_READER_H_
