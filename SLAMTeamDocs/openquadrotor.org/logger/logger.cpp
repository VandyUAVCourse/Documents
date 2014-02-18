#include <iostream>
#include <string>
#include <fstream>
#include <zlib.h>
#include <signal.h>
#include <ctime>
#include <sys/stat.h>

#include <ipcMessages/qc_imu_messages.h>
#include <ipcMessages/qc_laser_messages.h>
#include <ipcMessages/qc_odometry_messages.h>
#include <ipcMessages/qc_flightcontrol_messages.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <ipcInterfaces/qc_imu_interface.h>
#include <ipcInterfaces/qc_laser_interface.h>
#include <ipcInterfaces/qc_odometry_interface.h>
#include <ipcInterfaces/qc_flightcontrol_interface.h>
#include <utils_global/cmdargsreader.h>

using namespace std;

FILE *out;

qc_laser_laser_message lmsg1;
qc_laser_laser_message lmsg2;
qc_imu_imu_message imsg;
qc_odometry_odometry_message omsg;
qc_odometry_odometry_laserpoints_message olmsg;
qc_odometry_velocity_message vmsg;
qc_flightcontrol_flightcontrol_message fcmsg;
qc_flightcontrol_offboard_command_message ocmsg;

///FIXME multiple jpg picture message missing

bool silent = false;
bool stop = false;

string filename = "";
int jpg_counter = 0;

void sigterm_handler(int sgn __attribute__ ((unused))){
	stop = 1;
}

void qc_laser_laser1_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "[L1]");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &lmsg1, sizeof(lmsg1));
	fprintf(out, "LASER1 %d %d %f %f %d %d %d %ld %ld %d ", lmsg1.startStep, lmsg1.stopStep, lmsg1.startAngle, lmsg1.incrementAngle, lmsg1.clusterCount, lmsg1.status, lmsg1.internal_timestamp_hokuyo, lmsg1.timestamp_hokuyo_init_sec, lmsg1.timestamp_hokuyo_init_usec, lmsg1.num_ranges);
	for (int i=0; i<lmsg1.num_ranges; i++)
		fprintf(out,"%d ",lmsg1.ranges[i]);
	fprintf(out, "%ld %ld\n", lmsg1.timestamp_sec, lmsg1.timestamp_usec); 
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&lmsg1);
}

void qc_laser_laser2_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "[L2]");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &lmsg2, sizeof(lmsg2));
	fprintf(out, "LASER2 %d %d %f %f %d %d %d %ld %ld %d ", lmsg2.startStep, lmsg2.stopStep, lmsg2.startAngle, lmsg2.incrementAngle, lmsg2.clusterCount, lmsg2.status, lmsg2.internal_timestamp_hokuyo, lmsg2.timestamp_hokuyo_init_sec, lmsg2.timestamp_hokuyo_init_usec, lmsg2.num_ranges);
	for (int i=0; i<lmsg2.num_ranges; i++)
		fprintf(out,"%d ",lmsg2.ranges[i]);
	fprintf(out, "%ld %ld\n", lmsg2.timestamp_sec, lmsg2.timestamp_usec); 
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&lmsg2);
}

void qc_imu_imu_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "I");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &imsg, sizeof(imsg));
	fprintf(out,"IMU %f %f %f ", imsg.accX, imsg.accY, imsg.accZ);
	fprintf(out,"%f %f %f %f ", imsg.q0, imsg.q1, imsg.q2, imsg.q3);
	fprintf(out,"%f %f %f ", imsg.magX, imsg.magY, imsg.magZ);
	fprintf(out,"%f %f %f ", imsg.gyroX, imsg.gyroY, imsg.gyroZ);
	fprintf(out,"%ld %ld\n", imsg.timestamp_sec, imsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&imsg);
}

void qc_odometry_odometry_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "O");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &omsg, sizeof(omsg));
	fprintf(out,"ODOMETRY %f %f %f %d ", omsg.x, omsg.y, omsg.z, omsg.num_z_readings);
	for (int i=0; i<omsg.num_z_readings; i++)
		fprintf(out, "%f ", omsg.zr[i]);
	fprintf(out,"%f %f %f %f ", omsg.q0, omsg.q1, omsg.q2, omsg.q3);
	fprintf(out,"%f %f %f ", omsg.roll, omsg.pitch, omsg.yaw);
	fprintf(out,"%ld %ld\n", omsg.timestamp_sec, omsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&omsg);
}

void qc_odometry_odometry_laserpoints_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "[OL]");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &olmsg, sizeof(olmsg));
	fprintf(out,"ODOMETRY_LASERPOINTS %f %f %f %d ", olmsg.x, olmsg.y, olmsg.z, olmsg.num_z_readings);
	for (int i=0; i<olmsg.num_z_readings; i++)
		fprintf(out, "%f ", olmsg.zr[i]);
	fprintf(out,"%f %f %f %f ", olmsg.q0, olmsg.q1, olmsg.q2, olmsg.q3);
	fprintf(out,"%f %f %f ", olmsg.roll, olmsg.pitch, olmsg.yaw);
	fprintf(out,"%d ",olmsg.num_laser_readings);
	for (int i=0; i<olmsg.num_laser_readings; i++)
		fprintf(out, "%f %f ", olmsg.lx[i], olmsg.ly[i]);
	fprintf(out,"%ld %ld\n", olmsg.timestamp_sec, olmsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&olmsg);
}

void qc_odometry_velocity_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "V");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &vmsg, sizeof(vmsg));
	fprintf(out,"VELOCITY %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ", vmsg.x, vmsg.y, vmsg.z, vmsg.q0, vmsg.q1, vmsg.q2, vmsg.q3, vmsg.roll, vmsg.pitch, vmsg.yaw, vmsg.vx, vmsg.vy, vmsg.vz, vmsg.avx, vmsg.avy, vmsg.avz, vmsg.aix, vmsg.aiy, vmsg.aiz, vmsg.axtanp, vmsg.aytanr);
	fprintf(out,"%ld %ld\n", vmsg.timestamp_sec, vmsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&vmsg);
}

void qc_flightcontrol_flightcontrol_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "F");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &fcmsg, sizeof(fcmsg));
	fprintf(out,"FLIGHTCONTROL %d ", fcmsg.message_version);
	fprintf(out,"%d %d %d %d %d ", fcmsg.integral_nick, fcmsg.integral_roll, fcmsg.mean_acceleration_nick, fcmsg.mean_acceleration_roll, fcmsg.gyro_yaw);
	fprintf(out,"%d %d %d %d %d ", fcmsg.height_value, fcmsg.height_integral, fcmsg.mixture_value_acceleration, fcmsg.compass_value, fcmsg.battery_voltage);
	fprintf(out,"%d %d %d %d %d ", fcmsg.rc_connection, fcmsg.stick_roll, fcmsg.engine_front, fcmsg.engine_back, fcmsg.engine_left);
	fprintf(out,"%d %d %d %d %d ", fcmsg.engine_right, fcmsg.mean_acceleration_z, fcmsg.stick_yaw, fcmsg.stick_thrust, fcmsg.stick_pitch);
	fprintf(out,"%d %d %d %d %d ", fcmsg.servo, fcmsg.nick, fcmsg.roll, fcmsg.autonomous_enabled, fcmsg.gcm_failures);
	fprintf(out,"%d %d %d %d %d ", fcmsg.gcm_delta_roll, fcmsg.gcm_delta_pitch, fcmsg.gcm_delta_yaw, fcmsg.gcm_delta_thrust, fcmsg.measurement_roll);
	fprintf(out,"%d %d %ld %ld\n", fcmsg.calc_stick_pitch, fcmsg.calc_stick_roll, fcmsg.timestamp_sec, fcmsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&fcmsg);
}

void qc_flightcontrol_offboard_command_message_handler(MSG_INSTANCE mi, BYTE_ARRAY callData, void* clientData){
	(void) clientData; //no warning
	IPC_RETURN_TYPE err = IPC_OK;
	if (!silent)
		fprintf(stderr, "C");
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(mi), callData, &ocmsg, sizeof(ocmsg));
	fprintf(out,"OFFBOARDCOMMAND ");
	fprintf(out,"%d %d %d %d ", ocmsg.cmd_delta_roll, ocmsg.cmd_delta_roll_duration, ocmsg.cmd_delta_pitch, ocmsg.cmd_delta_pitch_duration);
	fprintf(out,"%d %d %d %d ", ocmsg.cmd_delta_yaw, ocmsg.cmd_delta_yaw_duration, ocmsg.cmd_delta_thrust, ocmsg.cmd_delta_thrust_duration);
	fprintf(out,"%ld %ld\n", ocmsg.timestamp_sec, ocmsg.timestamp_usec);
	IPC_freeByteArray(callData);
	IPC_freeDataElements(IPC_msgInstanceFormatter(mi),&ocmsg);
}

void write_initial_comments(){
	///write the comments to the file
	fprintf(out,"#Messages [ File Format Version 1.0 ]\n");
	///laser
	fprintf(out,"#LASER1 <startStep> <stopStep> <startAngle> <incrementAngle> <clusterCount> ");
	fprintf(out,"<status> <internal_timestamp_hokuyo> <timestamp_hokuyo_init_sec> ");
	fprintf(out,"<timestamp_hokuyo_init_usec> <num_ranges> <ranges 1...n> ");
	fprintf(out,"<timestamp_sec> <timestamp_usec> \n");
	
	fprintf(out,"#LASER2 <startStep> <stopStep> <startAngle> <incrementAngle> <clusterCount> ");
	fprintf(out,"<status> <internal_timestamp_hokuyo> <timestamp_hokuyo_init_sec> ");
	fprintf(out,"<timestamp_hokuyo_init_usec> <num_ranges> <ranges 1...n> ");
	fprintf(out,"<timestamp_sec> <timestamp_usec> \n");
	///imu
	fprintf(out,"#IMU <accX> <accY> <accZ> <q0> <q1> <q2> <q3> <magX> <magY> <magZ> <gyroX> ");
	fprintf(out,"<gyroY> <gyroZ> <timestamp_sec> <timestamp_usec>\n");
	
	
	fprintf(out,"#ODOMETRY <x> <y> <mean z> <num_z_readings> <z 1..n> <q0> <q1> <q2> <q3>  <roll> <pitch> <yaw>  <timestamp_sec> <timestamp_usec>\n");
	fprintf(out,"#ODOMETRY_LASERPOINTS <x> <y> <mean z> <num_z_readings> <z 1..n> <q0> <q1> <q2> <q3>  <roll> <pitch> <yaw> ");
	fprintf(out,"<num_laser_readings> < <laserpointX,laserpointY> 1..n> <timestamp_sec> <timestamp_usec>\n");
	fprintf(out,"#FLIGHTCONTROL <message_version> <integral_nick> <integral_roll> <mean_acceleration_nick> <mean_acceleration_roll> <gyro_yaw> <height_value> <height_integral> <mixture_value_acceleration> <compass_value> <battery_voltage> <rc_connection> <stick_roll> <engine_front> <engine_back> <engine_left> <engine_right> <mean_acceleration_z> <stick_yaw> <stick_thrust> <stick_pitch> <servo> <nick> <roll> <autonomous_enabled> <gcm_failures> <gcm_delta_roll> <gcm_delta_pitch> <gcm_delta_yaw> <gcm_delta_thrust> <measurement_roll> <calc_stick_pitch> <calc_stick_roll> <timestamp_sec> <timestamp_usec>\n");
	fprintf(out,"#VELOCITY <x> <y> <z> <q0> <q1> <q2> <q3> <roll> <pitch> <yaw> <vx> <vy> <vz> <accX_from_velocity> <accY_from_velocity> <accZ_from_velocity> <accX_from_imu> <accY_from_imu> <accZ_from_imu> <accX_from_pitch> <accY_from_roll> <timestamp_sec> <timestamp_usec>\n");
	fprintf(out,"#OFFBOARDCOMMAND <cmd_delta_roll> <cmd_delta_roll_duration> <cmd_delta_pitch> <cmd_delta_pitch_duration> <cmd_delta_yaw> <cmd_delta_yaw_duration> <cmd_delta_thrust> <cmd_delta_thrust_duration> <timestamp_sec> <timestamp_usec>\n");
}


int main (int argc, char* argv[]){

	if (argc < 2){
		std::cerr << "Usage= " << argv[0] << std::endl;
		std::cerr << "          -o | -output <outputfilename>" << std::endl;
		std::cerr << "          -s | -silent (no output to screen)" << std::endl;
		std::cerr << "          -g | -generateFilename " << std::endl;
		std::cerr << "          -m | -message <string without blanks> " << std::endl;
		std::cerr << std::endl;
		exit(1);
	}
	
	lmsg1.num_ranges = 0;
	lmsg1.ranges = 0;
	lmsg2.num_ranges = 0;
	lmsg2.ranges = 0;
	omsg.num_z_readings = 0;
	omsg.zr = 0;
	olmsg.num_z_readings = 0;
	olmsg.zr = 0;
	olmsg.num_laser_readings = 0;
	olmsg.lx = 0;
	olmsg.ly = 0;
	CmdArgsReader commandline(argc, argv);
	bool generateFilename = false;
	string message = "";
	commandline.getStringValue("-o","-output",&filename);
	commandline.getFlagValue("-s","-silent",&silent);
	commandline.getFlagValue("-g","-generateFilename",&generateFilename);
	commandline.getStringValue("-m","-message",&message);
	string directory_name = "";
	
	if (commandline.printUnusedArguments()){
		exit(1);
	}
	if (generateFilename){
		char buffer[256];
		time_t ct = time(NULL);
		strftime( buffer, 256, "log_%y%m%d_%H%M%S.log", localtime((const time_t*) &ct));
		char filenamebuffer[300];
		char directory_name_buffer[300];
		if (message!= "")
			sprintf(directory_name_buffer,"logs/%s.%s.dir",buffer, message.c_str());
		else
			sprintf(directory_name_buffer,"logs/%s.dir",buffer);
		directory_name = directory_name_buffer;
		sprintf(filenamebuffer,"%s/%s",directory_name.c_str(),buffer);
		filename = filenamebuffer;
		
		std::cerr << "Creating Directory: " << directory_name << std::endl;
		if (mkdir(directory_name.c_str(),0777)){
			std::cerr << " Error: unable to create directory " << directory_name << std::endl;
			exit(-1);
		}
		
		std::cerr << "Creating Filename: " << filename << std::endl;
	}

	if (filename == ""){
		std::cerr << "#unspecified output filename!" << std::endl;
		std::cerr << "use -g for automatically generating filenames" << std::endl;
		exit(1);
	}
	
	signal(SIGINT,sigterm_handler);
	out = fopen(filename.c_str(),"w");
	
	if (!out){
		std::cerr << "#Error: unable to open file \"" << filename << "\"" << std::endl;
		exit(1);
	}
	
	if (qc_ipc_connect(argv[0]) <= 0){
		exit(-1);
	}
	
	write_initial_comments();
	qc_laser_subscribe_laser1_message(qc_laser_laser1_message_handler,10,NULL);
	qc_laser_subscribe_laser2_message(qc_laser_laser2_message_handler,10,NULL);
	qc_imu_subscribe_imu_message(qc_imu_imu_message_handler,10,NULL);
	qc_odometry_subscribe_odometry_message(qc_odometry_odometry_message_handler, 10, NULL);
	qc_odometry_subscribe_odometry_laserpoints_message(qc_odometry_odometry_laserpoints_message_handler, 10, NULL);
	qc_odometry_subscribe_velocity_message(qc_odometry_velocity_message_handler, 10, NULL);
	qc_flightcontrol_subscribe_flightcontrol_message(qc_flightcontrol_flightcontrol_message_handler, 10, NULL);
	qc_flightcontrol_subscribe_offboard_command_message(qc_flightcontrol_offboard_command_message_handler, 10, NULL);
	
	while (!stop)
		IPC_listen(1);
	
	fclose(out);
	
	return 0;
}


