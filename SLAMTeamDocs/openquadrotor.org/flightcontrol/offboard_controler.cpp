#include <ipcMessages/qc_flightcontrol_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_flightcontrol_interface.h>
#include <utils_global/cmdargsreader.h>
#include <signal.h>
#include "gumstix2copter.cpp"
#include <sys/time.h>

qc_flightcontrol_offboard_command_message ocmsg;
gumstix_2_copter_message gcm;
bool newCommandMessageAvailable = false;
int commandMessageDuration = 50;
bool stop = false;
int msgcounter = 0;

void sigterm_handler(int sgn __attribute__ ((unused))){
	stop = true;
}

void qc_flightcontrol_offboard_command_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &ocmsg, sizeof(ocmsg));
	if (err != IPC_OK)
		fprintf(stderr, "#Error!\n");
	ocmsg.cmd_delta_roll = ocmsg.cmd_delta_roll > 127 ? 127 : ocmsg.cmd_delta_roll;
	ocmsg.cmd_delta_roll = ocmsg.cmd_delta_roll < -127 ? -127 : ocmsg.cmd_delta_roll;
	ocmsg.cmd_delta_roll_duration = ocmsg.cmd_delta_roll_duration > 127 ? 127 : ocmsg.cmd_delta_roll_duration;
	ocmsg.cmd_delta_roll_duration = ocmsg.cmd_delta_roll_duration < 0 ? 0 : ocmsg.cmd_delta_roll_duration;
	
	ocmsg.cmd_delta_pitch = ocmsg.cmd_delta_pitch > 127 ? 127 : ocmsg.cmd_delta_pitch;
	ocmsg.cmd_delta_pitch = ocmsg.cmd_delta_pitch < -127 ? -127 : ocmsg.cmd_delta_pitch;
	ocmsg.cmd_delta_pitch_duration = ocmsg.cmd_delta_pitch_duration > 127 ? 127 : ocmsg.cmd_delta_pitch_duration;
	ocmsg.cmd_delta_pitch_duration = ocmsg.cmd_delta_pitch_duration < 0 ? 0 : ocmsg.cmd_delta_pitch_duration;
	
	ocmsg.cmd_delta_yaw = ocmsg.cmd_delta_yaw > 127 ? 127 : ocmsg.cmd_delta_yaw;
	ocmsg.cmd_delta_yaw = ocmsg.cmd_delta_yaw < -127 ? -127 : ocmsg.cmd_delta_yaw;
	ocmsg.cmd_delta_yaw_duration = ocmsg.cmd_delta_yaw_duration > 127 ? 127 : ocmsg.cmd_delta_yaw_duration;
	ocmsg.cmd_delta_yaw_duration = ocmsg.cmd_delta_yaw_duration < 0 ? 0 : ocmsg.cmd_delta_yaw_duration;
	
	ocmsg.cmd_delta_thrust = ocmsg.cmd_delta_thrust > 127 ? 127 : ocmsg.cmd_delta_thrust;
	ocmsg.cmd_delta_thrust = ocmsg.cmd_delta_thrust < -127 ? -127 : ocmsg.cmd_delta_thrust;
	ocmsg.cmd_delta_thrust_duration = ocmsg.cmd_delta_thrust_duration > 127 ? 127 : ocmsg.cmd_delta_thrust_duration;
	ocmsg.cmd_delta_thrust_duration = ocmsg.cmd_delta_thrust_duration < 0 ? 0 : ocmsg.cmd_delta_thrust_duration;
	
	newCommandMessageAvailable = true;
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&ocmsg);	
}

int main (int argc, char* argv[]){
	bool noRoll = false;
	bool noPitch = false;
	bool noYaw = false;
	bool noThrust = false;
	bool debug = false;
	string device = "";
	CmdArgsReader commandline(argc, argv);
	commandline.getFlagValue("-nr","-noRoll",&noRoll);
	commandline.getFlagValue("-np","-noPitch",&noPitch);
	commandline.getFlagValue("-ny","-noYaw",&noYaw);
	commandline.getFlagValue("-nt","-noThrust",&noThrust);
	commandline.getFlagValue("-debug",&debug);
	commandline.getStringValue("-d","-device",&device);
	
	if (commandline.printUnusedArguments())
		return 0;
	
	Gumstix2Copter g2c(device.c_str(), debug);
	
	if (qc_ipc_connect(argv[0]) <= 0)
		return 0;
	signal(SIGINT,sigterm_handler);
	qc_flightcontrol_subscribe_offboard_command_message (qc_flightcontrol_offboard_command_message_handler, 1, NULL);
	
	while (!stop){
		if (newCommandMessageAvailable){
			newCommandMessageAvailable = false;
			gcm.cmd_delta_roll = (int8_t)ocmsg.cmd_delta_roll;
			gcm.cmd_delta_roll_duration = (int8_t)ocmsg.cmd_delta_roll_duration;
			gcm.cmd_delta_pitch = (int8_t)ocmsg.cmd_delta_pitch;
			gcm.cmd_delta_pitch_duration = (int8_t)ocmsg.cmd_delta_pitch_duration;
			gcm.cmd_delta_yaw = (int8_t)ocmsg.cmd_delta_yaw;
			gcm.cmd_delta_yaw_duration = (int8_t)ocmsg.cmd_delta_yaw_duration;
			gcm.cmd_delta_thrust = (int8_t)ocmsg.cmd_delta_thrust;
			gcm.cmd_delta_thrust_duration = (int8_t)ocmsg.cmd_delta_thrust_duration;
			if (noRoll){
				gcm.cmd_delta_roll = 0;
				gcm.cmd_delta_roll_duration = 0;	
			}
			if (noPitch){
				gcm.cmd_delta_pitch = 0;
				gcm.cmd_delta_pitch_duration = 0;
			}
			if (noYaw){
				gcm.cmd_delta_yaw = 0;
				gcm.cmd_delta_yaw_duration = 0;
			}
			if (noThrust){
				gcm.cmd_delta_thrust = 0;
				gcm.cmd_delta_thrust_duration = 0;
			}
			/*
			///send stuff
			struct timeval t;
			gettimeofday(&t, NULL);
			float dt = (t.tv_sec - ocmsg.timestamp_sec);
			dt += 1e-6 * (t.tv_usec - ocmsg.timestamp_usec);
			fprintf(stderr,"latency= %f\n",dt);
			*/
			if (!g2c.sendCommand(gcm)){
				fprintf(stderr,"F");
			} 
			if (!((msgcounter++)%10))
				fprintf(stderr,".");
		}
		IPC_listen(1);
		usleep(1000);
	}
	return 0;
}

