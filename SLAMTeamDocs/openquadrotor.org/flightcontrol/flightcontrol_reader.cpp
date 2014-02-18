#include <iostream>
#include <string>
using std::cout;

#include <libserial-0.5.2/SerialStream.h>
using namespace LibSerial ;

#include <sys/time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <ipcMessages/qc_flightcontrol_messages.h>
#include <ipcInterfaces/qc_flightcontrol_interface.h>
#include <ipcInterfaces/qc_ipc_interface.h>
#include <utils_global/cmdargsreader.h>

void decode64(unsigned char* in_arr, int offset, int len, unsigned char* out_arr) {
	int ptrIn=offset; 
	int a,b,c,d,x,y,z;
	int ptr=0;
	while(len!=0)
	{
		a = in_arr[ptrIn++] - '=';
		b = in_arr[ptrIn++] - '=';
		c = in_arr[ptrIn++] - '=';
		d = in_arr[ptrIn++] - '=';
 
		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if((len--)!=0) out_arr[ptr++] = x; else break;
		if((len--)!=0) out_arr[ptr++] = y; else break;
		if((len--)!=0) out_arr[ptr++] = z; else break;
	}
}

void process_message(unsigned char* buffer, int len, qc_flightcontrol_flightcontrol_message* msg){
	if (! msg)
		return;
	unsigned char outbuffer[512];
	decode64(buffer,0,len,outbuffer);
	int val[32];
	for (int i=0; i<32; i++){
		val[i] = (int16_t) ((outbuffer[2*i+3] << 8 )| outbuffer[2*i+2]);
	}
	msg->message_version = 69; //0.69
	msg->integral_nick                       = val[0];
	msg->integral_roll                       = val[1];
	msg->mean_acceleration_nick              = val[2];
	msg->mean_acceleration_roll              = val[3];
	msg->gyro_yaw                            = val[4];
	msg->height_value                        = val[5];
	msg->height_integral                     = val[6];
	msg->mixture_value_acceleration          = val[7];
	msg->compass_value                       = val[8];
	msg->battery_voltage                     = val[9];
	
	msg->rc_connection                       = val[10];
	msg->stick_pitch                         = val[11];
	msg->engine_front                        = val[12];
	msg->engine_back                         = val[13];
	msg->engine_left                         = val[14];
	msg->engine_right                        = val[15];
	msg->mean_acceleration_z                 = val[16];
	msg->stick_yaw         						  = val[17];
	msg->stick_thrust                        = val[18];
	msg->stick_roll                          = val[19];
	
	msg->servo                               = val[20];
	msg->nick                                = val[21];
	msg->roll                                = val[22];
	msg->autonomous_enabled                  = val[23];
	msg->gcm_failures                        = val[24];
	msg->gcm_delta_roll                      = val[25];
	msg->gcm_delta_pitch                     = val[26];
	msg->gcm_delta_yaw                       = val[27];
	msg->gcm_delta_thrust                    = val[28];
	msg->measurement_roll	                 = val[29];

	msg->calc_stick_pitch                    = val[30];
	msg->calc_stick_roll                     = val[31];
}



int main(int argc, char* argv[]){
	std::string device;
	qc_flightcontrol_flightcontrol_message msg;
	if (argc < 2){
		std::cerr << "Usage: " << argv[0] << std::endl;
		std::cerr << "                         -d | -device <pathToDevice>  {path to serial input}" << std::endl;
		return 0;
	}

	if (qc_ipc_connect(argv[0]) <= 0){
		return 0;
	}
	CmdArgsReader commandline(argc,argv);
	commandline.getStringValue("-d","-device",&device);
	if (commandline.printUnusedArguments()){
		return 0;
	}
	unsigned char buffer[512];
	for (int i=0; i<512; i++)
		buffer[i] = '+';
	
	SerialStream fd(device.c_str());
	if (!fd.good()){
		std::cerr << "Error : unable to open " << device << std::endl;
		return -1;
	}
	fd.SetBaudRate(SerialStreamBuf::BAUD_57600);
	fd.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	fd.SetNumOfStopBits(1);
	fd.SetParity(SerialStreamBuf::PARITY_NONE);
	fd.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	char c;
	int i=0;
	struct timeval ts_old, ts_new;
	///avoid warning
	ts_old.tv_sec = 0;
	ts_old.tv_usec = 0;
	///
	uint k = 0;
	while (1){
		while (fd.rdbuf()->in_avail()>0){
			fd.get(c);
			if (c == '#'){
				gettimeofday(&ts_new,NULL);
				if (i == 94 && buffer[0] == '#' && buffer[1] == '\0' && buffer[2] == 'D')	{
					///check crc
					int chk = 0;
					int l=0;
					for (l=0; l<i-3; l++){
						chk += buffer[l];
					}
					chk %= 4096;
					if (buffer[l++] == ('=' + chk / 64) && 
						 buffer[l++] == ('=' + chk % 64) && 
						 buffer[l++] == '\r') {
						process_message(buffer+3, i-3,&msg);
						msg.timestamp_sec = ts_old.tv_sec;
						msg.timestamp_usec = ts_old.tv_usec;
						qc_flightcontrol_publish_flightcontrol_message(&msg);
						if (!(k%10))
							std::cerr << "." << std::flush;
						k++;
					} else {
						std::cerr << "F" << std::flush;
					}
				}
				ts_old = ts_new;
				
				i=0;
				buffer[i++]=(unsigned char)c;
			} else {
				buffer[i++]=(unsigned char)c;
			}
		}
 		IPC_listen(1);	
		usleep(100);
	}
}

