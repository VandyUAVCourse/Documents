#include <ipc/ipc.h>
#include <ipcMessages/qc_laser_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <cmath>
#include <cstdlib>

qc_laser_laser_message lasermsg;
using namespace std;
void qc_laser_laser1_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
 	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (lasermsg.ranges){
		delete[] lasermsg.ranges;
		lasermsg.ranges = 0;
	}
	fprintf(stderr, "L");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &lasermsg, sizeof(lasermsg));
	
	int i;
	cout << "set size ratio -1" << endl;
	cout << "plot [-5600:5600][-5600:5600]'-' w p" << endl;
	for (i=0; i<lasermsg.num_ranges; i++){
		double alpha=lasermsg.startAngle + i * lasermsg.incrementAngle;
		cout << lasermsg.ranges[i] * cos(alpha) << " " << lasermsg.ranges[i] * sin (alpha) << endl; 
  }
  cout << "e" << endl << endl;
  IPC_freeByteArray(callData);
  IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&lasermsg);
}


int main (int argc, char* argv[]){
	lasermsg.ranges = 0;
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(1);
	}
	qc_laser_subscribe_laser1_message(qc_laser_laser1_message_handler,10,NULL);
	
	IPC_dispatch();
	
	return 0;
}
