#include <ipcInterfaces/qc_ipc_interface.h>

int qc_ipc_connect(char* modulName){
	IPC_setVerbosity(IPC_Silent);
	char ipcName[512];
	if(strrchr(modulName, '/') != NULL) {
		modulName = strrchr(modulName, '/');
		modulName++;
	}
	snprintf(ipcName, 512, "%s-%d", modulName, getpid());
	std::cerr << "Connecting to IPC... " << std::flush;
	if (IPC_connect(ipcName)==IPC_OK){
		std::cerr << ESC << COLOR_SUCCESS << "SUCCESS" << ESC << COLOR_RESET << std::endl;
		return 1;
	} else {
		std::cerr << ESC << COLOR_FAILED << "FAILED" << ESC << COLOR_RESET << std::endl;
		return 0;
	}
}
