// MESSAGE THERMAL_SENSOR_1 PACKING

#define MAVLINK_MSG_ID_THERMAL_SENSOR_1 150

typedef struct __mavlink_thermal_sensor_1_t
{
 uint16_t temperature; ///< Temperature reading from the thermal sensor.
} mavlink_thermal_sensor_1_t;

#define MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN 2
#define MAVLINK_MSG_ID_150_LEN 2

#define MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC 54
#define MAVLINK_MSG_ID_150_CRC 54



#define MAVLINK_MESSAGE_INFO_THERMAL_SENSOR_1 { \
	"THERMAL_SENSOR_1", \
	1, \
	{  { "temperature", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_thermal_sensor_1_t, temperature) }, \
         } \
}


/**
 * @brief Pack a thermal_sensor_1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature Temperature reading from the thermal sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermal_sensor_1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN];
	_mav_put_uint16_t(buf, 0, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#else
	mavlink_thermal_sensor_1_t packet;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMAL_SENSOR_1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
}

/**
 * @brief Pack a thermal_sensor_1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature Temperature reading from the thermal sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermal_sensor_1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN];
	_mav_put_uint16_t(buf, 0, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#else
	mavlink_thermal_sensor_1_t packet;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMAL_SENSOR_1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
}

/**
 * @brief Encode a thermal_sensor_1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thermal_sensor_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thermal_sensor_1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thermal_sensor_1_t* thermal_sensor_1)
{
	return mavlink_msg_thermal_sensor_1_pack(system_id, component_id, msg, thermal_sensor_1->temperature);
}

/**
 * @brief Encode a thermal_sensor_1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thermal_sensor_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thermal_sensor_1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_thermal_sensor_1_t* thermal_sensor_1)
{
	return mavlink_msg_thermal_sensor_1_pack_chan(system_id, component_id, chan, msg, thermal_sensor_1->temperature);
}

/**
 * @brief Send a thermal_sensor_1 message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature Temperature reading from the thermal sensor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thermal_sensor_1_send(mavlink_channel_t chan, uint16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN];
	_mav_put_uint16_t(buf, 0, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
#else
	mavlink_thermal_sensor_1_t packet;
	packet.temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, (const char *)&packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, (const char *)&packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_thermal_sensor_1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, buf, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
#else
	mavlink_thermal_sensor_1_t *packet = (mavlink_thermal_sensor_1_t *)msgbuf;
	packet->temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, (const char *)packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN, MAVLINK_MSG_ID_THERMAL_SENSOR_1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMAL_SENSOR_1, (const char *)packet, MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE THERMAL_SENSOR_1 UNPACKING


/**
 * @brief Get field temperature from thermal_sensor_1 message
 *
 * @return Temperature reading from the thermal sensor.
 */
static inline uint16_t mavlink_msg_thermal_sensor_1_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a thermal_sensor_1 message into a struct
 *
 * @param msg The message to decode
 * @param thermal_sensor_1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_thermal_sensor_1_decode(const mavlink_message_t* msg, mavlink_thermal_sensor_1_t* thermal_sensor_1)
{
#if MAVLINK_NEED_BYTE_SWAP
	thermal_sensor_1->temperature = mavlink_msg_thermal_sensor_1_get_temperature(msg);
#else
	memcpy(thermal_sensor_1, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_THERMAL_SENSOR_1_LEN);
#endif
}
