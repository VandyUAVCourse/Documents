#ifndef _FLIGHTCONTROL_MESSAGES_H_
#define _FLIGHTCONTROL_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
	
typedef struct {
	int message_version;
	///0 - 9
	int integral_nick;
	int integral_roll;
	int mean_acceleration_nick;      ///MittelWert_AccNick  ///AccNick
	int mean_acceleration_roll;      ///MittelWert_AccRoll  ///AccRoll
	int gyro_yaw;                    ///Messwert-Gier       ///GyroGier
	int height_value;                ///Hoehenwert          ///Hoehenwert
	int height_integral;             ///Mess_Integral_Hoch/512  ///AccZ
	int mixture_value_acceleration;  ///GasMischAnteil    ///Gas
	int compass_value;               ///KompassValue      ///KompassValue
	int battery_voltage;					///UBat              ///Spannung
	/// 10 - 19
	int rc_connection;               ///SenderOkay      ///Empfang
	int stick_roll;                  ///RAW funke roll input
	int engine_front;                ///MotorVorne  
	int engine_back;                 ///MotorHinten
	int engine_left;                 ///MotorLinks
	int engine_right;                ///MotorRechts
	int mean_acceleration_z;         ///MittelWert_Acc_Hoch     ///Acc_Z
	int stick_yaw;                    ///FIXME Not used in v069 ///
	int stick_thrust;							///FIXME Not used in v069
	int stick_pitch;         ///RAW funke pitch _INPUT
	/// 20 - 29
	int servo;
	int nick; 
	int roll; 
	
	int autonomous_enabled;          ///Poti3 -> switch enabled?
	int gcm_failures;                ///# of not received messages
	int gcm_delta_roll; 					///GCM Roll Message
	int gcm_delta_pitch; 				///GCM Pitch Message
	int gcm_delta_yaw; 				   ///GCM Yaw Message
	int gcm_delta_thrust; 				///GCM Pitch Message
	int measurement_roll;            ///AusgleichRoll 
	/// 30 - 31
// 	int gps_nick;
// 	int gps_roll;
	int calc_stick_pitch;
	int calc_stick_roll;
	
	long timestamp_sec;
	long timestamp_usec;
} qc_flightcontrol_flightcontrol_message;

#define QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_NAME "qc_flightcontrol_flightcontrol_message"
#define QC_FLIGHTCONTROL_FLIGHTCONTROL_MESSAGE_FMT  "{int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, long, long}"


typedef struct { ///simulate RC commands this is used for sending commands from pc to mircocontroller
	int8_t cmd_delta_roll;
	int8_t cmd_delta_roll_duration;
	int8_t cmd_delta_pitch;
	int8_t cmd_delta_pitch_duration;
	int8_t cmd_delta_yaw;
	int8_t cmd_delta_yaw_duration;
	int8_t cmd_delta_thrust;
	int8_t cmd_delta_thrust_duration;
} gumstix_2_copter_message;


typedef struct {
	int cmd_delta_roll;
	int cmd_delta_roll_duration;
	int cmd_delta_pitch;
	int cmd_delta_pitch_duration;
	int cmd_delta_yaw;
	int cmd_delta_yaw_duration;
	int cmd_delta_thrust;
	int cmd_delta_thrust_duration;	
	long timestamp_sec;
	long timestamp_usec;
} qc_flightcontrol_offboard_command_message;

#define QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_NAME "qc_flightcontrol_offboard_command_message"
#define QC_FLIGHTCONTROL_OFFBOARD_COMMAND_MESSAGE_FMT "{int, int, int, int, int, int, int, int, long, long}"


#ifdef __cplusplus
}
#endif


#endif //_FLIGHTCONTROL_MESSAGES_H_
