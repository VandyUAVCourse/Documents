#include "gumstix2copter.h"


Gumstix2Copter::Gumstix2Copter(string device, bool _debug) : fd(device.c_str()){
	if (_debug)
		return;
	debug = _debug;
	if (!fd.good()){
		std::cerr << "Error : unable to open " << device << std::endl;
		exit(1);
	}
	fd.SetBaudRate(SerialStreamBuf::BAUD_57600);
	fd.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	fd.SetNumOfStopBits(1);
	fd.SetParity(SerialStreamBuf::PARITY_NONE);
	fd.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
}

bool Gumstix2Copter::sendCommand(gumstix_2_copter_message msg){
	char cmd[11] = {0xfa,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfb};
	int8_t chksum = 0x00;
	///security
	msg.cmd_delta_roll = (msg.cmd_delta_roll > MAX_ROLL)? MAX_ROLL : msg.cmd_delta_roll;
	msg.cmd_delta_roll = (msg.cmd_delta_roll < MIN_ROLL)? MIN_ROLL : msg.cmd_delta_roll;
	
	msg.cmd_delta_pitch = (msg.cmd_delta_pitch > MAX_PITCH)? MAX_PITCH : msg.cmd_delta_pitch;
	msg.cmd_delta_pitch = (msg.cmd_delta_pitch < MIN_PITCH)? MIN_PITCH : msg.cmd_delta_pitch;
	
	msg.cmd_delta_yaw = (msg.cmd_delta_yaw > MAX_YAW)? MAX_YAW : msg.cmd_delta_yaw;
	msg.cmd_delta_yaw = (msg.cmd_delta_yaw < MIN_YAW)? MIN_YAW : msg.cmd_delta_yaw;
	
	msg.cmd_delta_thrust = (msg.cmd_delta_thrust > MAX_THRUST)? MAX_THRUST : msg.cmd_delta_thrust;
	msg.cmd_delta_thrust = (msg.cmd_delta_thrust < MIN_THRUST)? MIN_THRUST : msg.cmd_delta_thrust;
	
	cmd[1] = (int8_t) msg.cmd_delta_roll;
	cmd[2] = (int8_t) msg.cmd_delta_roll_duration;
	cmd[3] = (int8_t) msg.cmd_delta_pitch;
	cmd[4] = (int8_t) msg.cmd_delta_pitch_duration;
	cmd[5] = (int8_t) msg.cmd_delta_yaw;
	cmd[6] = (int8_t) msg.cmd_delta_yaw_duration;
	cmd[7] = (int8_t) msg.cmd_delta_thrust;
	cmd[8] = (int8_t) msg.cmd_delta_thrust_duration;
	for (int i=1; i<=8; i++)
		chksum += cmd[i];
	cmd[9] = chksum;
	if (debug)
		return true;
	if (fd.good()){
		fd.write(cmd,11);
		if (fd.good())
			return true;
		else
			return false;
	} else
		return false;
	
}

/*
bool Gumstix2Copter::sendCommand(const gumstix_2_copter_message& msg){
	char cmd[12] = {0xaa,0x0f,0xf0,0x0f,0xf0,0x0f,0xf0,0x0f,0xf0,0x0f,0xf0,0xbb};
	int8_t chksum = 0x00;
	cmd[1] = (int8_t)(  msg.cmd_delta_roll   | cmd[1] );
	cmd[2] = (int8_t)(  msg.cmd_delta_roll   | cmd[2] );
	cmd[3] = (int8_t)(  msg.cmd_delta_pitch  | cmd[3] );
	cmd[4] = (int8_t)(  msg.cmd_delta_pitch  | cmd[4] );
	cmd[5] = (int8_t)(  msg.cmd_delta_yaw    | cmd[5] );
	cmd[6] = (int8_t)(  msg.cmd_delta_yaw    | cmd[6] );
	cmd[7] = (int8_t)(  msg.cmd_delta_thrust | cmd[7] );
	cmd[8] = (int8_t)(  msg.cmd_delta_thrust | cmd[8] );
	for (int i=1; i<=8; i++)
		chksum += cmd[i];
	cmd[9]  = (int8_t)( chksum | cmd[9] );
	cmd[10] = (int8_t)( chksum | cmd[10] );
	
	
	if (fd.good()){
		fd.write(cmd,12);
		if (fd.good())
			return true;
		else
			return false;
	} else
		return false;
	
}
*/
void Gumstix2Copter::close(){
	if (fd.good())
		fd.Close();
}
