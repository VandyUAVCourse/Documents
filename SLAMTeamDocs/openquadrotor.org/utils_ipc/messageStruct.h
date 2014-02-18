#ifndef _MESSAGE_STUCT_H_
#define _MESSAGE_STUCT_H_
#include <string>
using namespace std;

struct MessageEntry{
	string pre_defined_name;
	string post_defined_name;
	string pre_defined_fmt;
	string post_defined_type;
	MessageEntry();
};

#endif // _MESSAGE_STUCT_H_
