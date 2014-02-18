#ifndef _CMD_ARGS_READER_H_
#define _CMD_ARGS_READER_H_
/*!
 * Commandline Arguments Reader
 * This class provides simple methods for easily read parameters from commandline
 * e.g. start program with myprogram -f filename -someFlag
 * in your code you have:
 * int main(int argc, char* argv[]){
 *   CmdArgsReader commandline(argc,argv);
 *   string filename;
 *   bool importantFlag;
 *   commandline.getStringValue("-f","--f",&filename);
 *   commandline.getFlag("-someFlag",&importantFlag);
 *   if (commandline.printUnusedArguments())
 * 	  exit(-1);
 * }
 */

#include <cstring>
#include <vector>
#include <iostream>
#include <cstdarg>
#include <cstdlib>
#include <stdint.h>

class CmdArgsReader{
	public:
		/*!
		* Constructor
		* @param argc num of arguments
		* @param argv array of arguments
		*/
		CmdArgsReader (int argc, char* argv[]);
		
		/*!
		* enable or disable ouput to std::cerr
		* @param _displayWarning enable/disable "warning" output
		* @param _displayFoundVars enable/disable "found parameter" output
		*/
		void set(bool _displayWarnings, bool _displayFoundVars);
		
		/*!
		* if (displayWarning == true) (see function set(..)) then
		* outputs warnings for all unused arguments
		* @param return the number of unused Arguments
		*/
		int printUnusedArguments();
				
		/*!
		* @param numNames number of Names parameters
		* @param numVars number of Variable* parameters
		* @param ... list of <numNames> names and <numVars> int*
		* @return 1 if success, 0 if failure occured.
		* prints Warnings if enabled
		* example
		* int a,b;
		* getIntValues(3,2,"-intValues","--i","-i",&a,&b)
		* will search for a command argument starting with "-intValues","--i" or "-i"
		* if found, the two successice values will be written into a and b.
		* Note: No validy check is made, so there will be no error if invoking
		* the program with ./program -intValues 17 notANumber
		* since the typecast will set a to 17 and b to 0
		*/
		int getIntValue(int numNames, int numVars, ...);
		
		/*!
		* @param name searchString
		* @param var pointer to write value to
		* @return 1 if success, 0 otherwise
		* this is a short version of
		* getIntValue(1,1,name,var);
		*/
		int getIntValue(const char* name, int* var);
		
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getIntValue(2,1,name1,name2,var);
		 */
		int getIntValue(const char* name1, const char* name2, int* var);
		
		/*!
		 * @param numNames number of Names parameters
		 * @param numVars number of Variable* parameters
		 * @param ... list of <numNames> names and <numVars> double*
		 * @return 1 if success, 0 if failure occured.
		 * prints Warnings if enabled
		 * example
		 * double a,b;
		 * getDoubleValues(3,2,"-doubleValues","--d","-d",&a,&b)
		 * will search for a command argument starting with "-doubleValues","--d" or "-d"
		 * if found, the two successice values will be written into a and b.
		 * Note: No validy check is made, so there will be no error if invoking
		 * the program with ./program --d 17 notANumber
		 * since the typecast will set a to 17 and b to 0
		 */
		int getDoubleValue(int numNames, int numVars, ...);
		/*!
		 * @param name searchString
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getDobuleValue(1,1,name,var);
		 */
		int getDoubleValue(const char* name, double* var);
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getDoubleValue(2,1,name1,name2,var);
		 */
		int getDoubleValue(const char* name1, const char* name2, double* var);
		
		
		/*!
		 * @param numNames number of Names parameters
		 * @param numVars number of Variable* parameters
		 * @param ... list of <numNames> names and <numVars> double*
		 * @return 1 if success, 0 if failure occured.
		 * prints Warnings if enabled
		 * example
		 * double a,b;
		 * getFloatValues(3,2,"-floatValues","--f","-f",&a,&b)
		 * will search for a command argument starting with "-floatValues","--f" or "-f"
		 * if found, the two successice values will be written into a and b.
		 * Note: No validy check is made, so there will be no error if invoking
		 * the program with ./program --f 17 notANumber
		 * since the typecast will set a to 17 and b to 0
		 */
		int getFloatValue(int numNames, int numVars, ...);
		/*!
		 * @param name searchString
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getDobuleValue(1,1,name,var);
		 */
		int getFloatValue(const char* name, float* var);
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getDoubleValue(2,1,name1,name2,var);
		 */
		int getFloatValue(const char* name1, const char* name2, float* var);
		
		
		/*!
		 * @param numNames number of Names parameters
		 * @param numVars number of Variable* parameters
		 * @param ... list of <numNames> names and <numVars> char*
		 * @return 1 if success, 0 if failure occured.
		 * prints Warnings if enabled
		 * example
		 * char a[1024],b[1024];
		 * getDoubleValues(3,2,"-stringValues","--s","-s",a,b)
		 * will search for a command argument starting with "-stringValues","--s" or "-s"
		 * if found, the two successice values will be written into a and b.
		 * Note: No validy check is made regarding the array size, so make sure a and b are big enough!
		 */
		int getStringValue(int numNames, int numVars, ...);
		
		/*!
		 * @param name searchString
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getStringValue(1,1,name,var);
		 */
		int getStringValue(const char* name, std::string* var);
		
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getStringValue(2,1,name1,name2,var);
		 */
		int getStringValue(const char* name1, const char* name2, std::string* var);
	
		/*!
		 * @param numNames number of Names parameters
		 * @param numVars number of Variable* parameters
		 * @param ... list of <numNames> names and <numVars> bool*
		 * @return 1 if success, 0 if failure occured.
		 * prints Warnings if enabled
		 * example
		 * bool a,b;
		 * getBoolValues(3,2,"-boolValues","--b","-b",a,b)
		 * will search for a command argument starting with "-boolValues","--b" or "-b"
		 * if found, the two successice values will be written into a and b.
		 * Note: No validy check is made, so there will be no error if invoking
		 * the program with ./program --b 17 notABool
		 * since the typecast will set a to false and b to false
		 * values mapped to true are {1,true,TRUE} every lower/uppercase writing of the word True
		 * values mapped to false: all other :)
		 */
		int getBoolValue(int numNames, int numVars, ...);
		
		/*!
		 * @param name searchString
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getBoolValue(1,1,name,var);
		 */
		int getBoolValue(const char* name, bool* var);
		
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getBoolValue(2,1,name1,name2,var);
		 */
		int getBoolValue(const char* name1, const char* name2, bool* var);
		
		
		/*!
		 * @param numNames number of Names parameters
		 * @param var pointer to write value to
		 * @param ... list of <numNames> names 
		 * @return 1 if success, 0 if failure occured.
		 * prints Warnings if enabled
		 * example
		 * bool aa;
		 * getFlag(3,&aa,"-flagAlpha","--alpha","-a")
		 * will search for command argument "-flagAlpha","--alpha" or "-a"
		 * if found, 'aa' will be set to true, otherwise 'aa' will be set to false.
		 */
		int getFlagValue(int numNames, bool* var, ...);
		
		/*!
		 * @param name searchString
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getFlag(1,var,name);
		 */
		int getFlagValue(const char* name, bool* var);
		
		/*!
		 * @param name1 searchString1
		 * @param name2 searchString2
		 * @param var pointer to write value to
		 * @return 1 if success, 0 otherwise
		 * this is a short version of
		 * getFlag(2,var,name1,name2);
		 */
		int getFlagValue(const char* name1, const char* name2, bool* var);
		
		
	private:
		std::vector<std::string> getValues(std::vector<std::string>& names, int& numValues);
		void begin_warning();
		void end_warning();
		void begin_info();
		void end_info();
		
		std::vector<std::string> values;
		std::vector<bool> usedValues;
		bool displayWarnings;
		bool displayFoundVars;
};

#endif //_CMD_ARGS_READER_H_

