// MESSAGE NAVI6D_DEBUG_OUTPUT PACKING

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT 151

typedef struct __mavlink_navi6d_debug_output_t
{
 double lat; ///< 
 double lon; ///< 
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float alt; ///< 
 float roll; ///< 
 float pitch; ///< 
 float yaw; ///< 
 uint8_t kalman_state_size; ///< 
 uint8_t kalman_meas_size; ///< 
} mavlink_navi6d_debug_output_t;

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN 38
#define MAVLINK_MSG_ID_151_LEN 38

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC 13
#define MAVLINK_MSG_ID_151_CRC 13



#define MAVLINK_MESSAGE_INFO_NAVI6D_DEBUG_OUTPUT { \
	"NAVI6D_DEBUG_OUTPUT", \
	9, \
	{  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_navi6d_debug_output_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_navi6d_debug_output_t, lon) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_navi6d_debug_output_t, time_boot_ms) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_navi6d_debug_output_t, alt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_navi6d_debug_output_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_navi6d_debug_output_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_navi6d_debug_output_t, yaw) }, \
         { "kalman_state_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_navi6d_debug_output_t, kalman_state_size) }, \
         { "kalman_meas_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_navi6d_debug_output_t, kalman_meas_size) }, \
         } \
}


/**
 * @brief Pack a navi6d_debug_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat 
 * @param lon 
 * @param alt 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @param kalman_state_size 
 * @param kalman_meas_size 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navi6d_debug_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, double lat, double lon, float alt, float roll, float pitch, float yaw, uint8_t kalman_state_size, uint8_t kalman_meas_size)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN];
	_mav_put_double(buf, 0, lat);
	_mav_put_double(buf, 8, lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, alt);
	_mav_put_float(buf, 24, roll);
	_mav_put_float(buf, 28, pitch);
	_mav_put_float(buf, 32, yaw);
	_mav_put_uint8_t(buf, 36, kalman_state_size);
	_mav_put_uint8_t(buf, 37, kalman_meas_size);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#else
	mavlink_navi6d_debug_output_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.time_boot_ms = time_boot_ms;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.kalman_state_size = kalman_state_size;
	packet.kalman_meas_size = kalman_meas_size;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
}

/**
 * @brief Pack a navi6d_debug_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat 
 * @param lon 
 * @param alt 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @param kalman_state_size 
 * @param kalman_meas_size 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navi6d_debug_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,double lat,double lon,float alt,float roll,float pitch,float yaw,uint8_t kalman_state_size,uint8_t kalman_meas_size)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN];
	_mav_put_double(buf, 0, lat);
	_mav_put_double(buf, 8, lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, alt);
	_mav_put_float(buf, 24, roll);
	_mav_put_float(buf, 28, pitch);
	_mav_put_float(buf, 32, yaw);
	_mav_put_uint8_t(buf, 36, kalman_state_size);
	_mav_put_uint8_t(buf, 37, kalman_meas_size);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#else
	mavlink_navi6d_debug_output_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.time_boot_ms = time_boot_ms;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.kalman_state_size = kalman_state_size;
	packet.kalman_meas_size = kalman_meas_size;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
}

/**
 * @brief Encode a navi6d_debug_output struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param navi6d_debug_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navi6d_debug_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_navi6d_debug_output_t* navi6d_debug_output)
{
	return mavlink_msg_navi6d_debug_output_pack(system_id, component_id, msg, navi6d_debug_output->time_boot_ms, navi6d_debug_output->lat, navi6d_debug_output->lon, navi6d_debug_output->alt, navi6d_debug_output->roll, navi6d_debug_output->pitch, navi6d_debug_output->yaw, navi6d_debug_output->kalman_state_size, navi6d_debug_output->kalman_meas_size);
}

/**
 * @brief Encode a navi6d_debug_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param navi6d_debug_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navi6d_debug_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_navi6d_debug_output_t* navi6d_debug_output)
{
	return mavlink_msg_navi6d_debug_output_pack_chan(system_id, component_id, chan, msg, navi6d_debug_output->time_boot_ms, navi6d_debug_output->lat, navi6d_debug_output->lon, navi6d_debug_output->alt, navi6d_debug_output->roll, navi6d_debug_output->pitch, navi6d_debug_output->yaw, navi6d_debug_output->kalman_state_size, navi6d_debug_output->kalman_meas_size);
}

/**
 * @brief Send a navi6d_debug_output message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat 
 * @param lon 
 * @param alt 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @param kalman_state_size 
 * @param kalman_meas_size 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_navi6d_debug_output_send(mavlink_channel_t chan, uint32_t time_boot_ms, double lat, double lon, float alt, float roll, float pitch, float yaw, uint8_t kalman_state_size, uint8_t kalman_meas_size)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN];
	_mav_put_double(buf, 0, lat);
	_mav_put_double(buf, 8, lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, alt);
	_mav_put_float(buf, 24, roll);
	_mav_put_float(buf, 28, pitch);
	_mav_put_float(buf, 32, yaw);
	_mav_put_uint8_t(buf, 36, kalman_state_size);
	_mav_put_uint8_t(buf, 37, kalman_meas_size);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
#else
	mavlink_navi6d_debug_output_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.time_boot_ms = time_boot_ms;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.kalman_state_size = kalman_state_size;
	packet.kalman_meas_size = kalman_meas_size;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_navi6d_debug_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, double lat, double lon, float alt, float roll, float pitch, float yaw, uint8_t kalman_state_size, uint8_t kalman_meas_size)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_double(buf, 0, lat);
	_mav_put_double(buf, 8, lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, alt);
	_mav_put_float(buf, 24, roll);
	_mav_put_float(buf, 28, pitch);
	_mav_put_float(buf, 32, yaw);
	_mav_put_uint8_t(buf, 36, kalman_state_size);
	_mav_put_uint8_t(buf, 37, kalman_meas_size);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
#else
	mavlink_navi6d_debug_output_t *packet = (mavlink_navi6d_debug_output_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->time_boot_ms = time_boot_ms;
	packet->alt = alt;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->kalman_state_size = kalman_state_size;
	packet->kalman_meas_size = kalman_meas_size;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NAVI6D_DEBUG_OUTPUT UNPACKING


/**
 * @brief Get field time_boot_ms from navi6d_debug_output message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_navi6d_debug_output_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field lat from navi6d_debug_output message
 *
 * @return 
 */
static inline double mavlink_msg_navi6d_debug_output_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field lon from navi6d_debug_output message
 *
 * @return 
 */
static inline double mavlink_msg_navi6d_debug_output_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field alt from navi6d_debug_output message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_output_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field roll from navi6d_debug_output message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_output_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitch from navi6d_debug_output message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_output_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw from navi6d_debug_output message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_output_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field kalman_state_size from navi6d_debug_output message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_navi6d_debug_output_get_kalman_state_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field kalman_meas_size from navi6d_debug_output message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_navi6d_debug_output_get_kalman_meas_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Decode a navi6d_debug_output message into a struct
 *
 * @param msg The message to decode
 * @param navi6d_debug_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_navi6d_debug_output_decode(const mavlink_message_t* msg, mavlink_navi6d_debug_output_t* navi6d_debug_output)
{
#if MAVLINK_NEED_BYTE_SWAP
	navi6d_debug_output->lat = mavlink_msg_navi6d_debug_output_get_lat(msg);
	navi6d_debug_output->lon = mavlink_msg_navi6d_debug_output_get_lon(msg);
	navi6d_debug_output->time_boot_ms = mavlink_msg_navi6d_debug_output_get_time_boot_ms(msg);
	navi6d_debug_output->alt = mavlink_msg_navi6d_debug_output_get_alt(msg);
	navi6d_debug_output->roll = mavlink_msg_navi6d_debug_output_get_roll(msg);
	navi6d_debug_output->pitch = mavlink_msg_navi6d_debug_output_get_pitch(msg);
	navi6d_debug_output->yaw = mavlink_msg_navi6d_debug_output_get_yaw(msg);
	navi6d_debug_output->kalman_state_size = mavlink_msg_navi6d_debug_output_get_kalman_state_size(msg);
	navi6d_debug_output->kalman_meas_size = mavlink_msg_navi6d_debug_output_get_kalman_meas_size(msg);
#else
	memcpy(navi6d_debug_output, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT_LEN);
#endif
}
