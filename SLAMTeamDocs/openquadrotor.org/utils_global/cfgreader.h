#ifndef _CFG_READER_H
#define _CFG_READER_H

#include <fstream>
#include <vector>
#include <stdint.h>
#include <cstdlib>
#include <iostream>

enum CfgReaderReturnValue {CfgReader_OK=0, CfgReader_GROUP_NOT_FOUND=1, CfgReader_VALUE_NOT_FOUND=2};

class CfgReader {
	public:
		/*!
		* Constructor
		*/
		CfgReader();
		/*!
		* Destructor
		*/
		~CfgReader();
		
		/*!
		* abort if group or value not found (default yes);
		@param displayOnError if true, cfgreader will display a status message if no value / group found. Default is true
		*/
		void setDisplayOnError(bool abortOnError);
		
		/*!
		* read parameters from given file
		* @param filename path to the cfg-file
		* @return 1 if successful, 0 if failure
		*/
		int openFile(const std::string filename);
		
		/*!
		* Returns the int values found for entry name in group "[group]"
		* @param group search for value in this group
		* @param name seach for name in the given group
		* @return vector with the found values (empty vector is returend, if no values found)
		*/
		std::vector<int> getIntValues (const std::string group, const std::string name, CfgReaderReturnValue& returnValue);
		
		/*!
		 * Returns the int values found for entry name in group "[group]"
		 * @param group search for value in this group
		 * @param name seach for name in the given group
		 * @return vector with the found values (empty vector is returend, if no values found)
		 */
		std::vector<double> getDoubleValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue);
		
		/*!
		 * Returns the int values found for entry name in group "[group]"
		 * @param group search for value in this group
		 * @param name seach for name in the given group
		 * @return vector with the found values (empty vector is returend, if no values found)
		 */
		std::vector<std::string> getStringValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue);
		
		/*!
		 * Returns the int values found for entry name in group "[group]"
		 * @param group search for value in this group
		 * @param name seach for name in the given group
		 * @return vector with the found values (empty vector is returend, if no values found)
		 */
		std::vector<bool> getBoolValues  (const std::string group, const std::string name, CfgReaderReturnValue& returnValue);
		
		
		/*!
		* Returns CfgReader_OK if group found else CfgReader_GROUP_NOT_FOUND
		@param group seach for this groupname
		@return CfgReader_OK if group tag found, else CfgReader_GROUP_NOT_FOUND
		*/
		CfgReaderReturnValue isGroupPresent(const std::string group);
		
 	private:
		void readValues(const std::string group, const std::string name, CfgReaderReturnValue& returnValue);
		std::ifstream in;
		std::vector<std::string> values;
		bool displayOnError;
};


#endif //_CFG_READER_H
