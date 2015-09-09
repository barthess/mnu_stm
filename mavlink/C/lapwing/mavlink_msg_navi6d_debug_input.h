// MESSAGE NAVI6D_DEBUG_INPUT PACKING

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT 150

typedef struct __mavlink_navi6d_debug_input_t
{
 double gnss_lat; ///< 
 double gnss_lon; ///< 
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float gnss_alt; ///< 
 float gnss_speed; ///< 
 float gnss_course; ///< 
 float gnss_v[3]; ///< 3 component speed vector
 float marg_dt; ///< 
 float marg_acc[3]; ///< 
 float marg_gyr[3]; ///< 
 float marg_mag[3]; ///< 
 float baro_alt; ///< 
 float odo_speed; ///< 
 uint8_t gnss_fresh; ///< bool flag
 uint8_t gnss_speed_type; ///< 
 uint8_t gnss_fix_type; ///< 
} mavlink_navi6d_debug_input_t;

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN 95
#define MAVLINK_MSG_ID_150_LEN 95

#define MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC 109
#define MAVLINK_MSG_ID_150_CRC 109

#define MAVLINK_MSG_NAVI6D_DEBUG_INPUT_FIELD_GNSS_V_LEN 3
#define MAVLINK_MSG_NAVI6D_DEBUG_INPUT_FIELD_MARG_ACC_LEN 3
#define MAVLINK_MSG_NAVI6D_DEBUG_INPUT_FIELD_MARG_GYR_LEN 3
#define MAVLINK_MSG_NAVI6D_DEBUG_INPUT_FIELD_MARG_MAG_LEN 3

#define MAVLINK_MESSAGE_INFO_NAVI6D_DEBUG_INPUT { \
	"NAVI6D_DEBUG_INPUT", \
	16, \
	{  { "gnss_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_navi6d_debug_input_t, gnss_lat) }, \
         { "gnss_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_navi6d_debug_input_t, gnss_lon) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_navi6d_debug_input_t, time_boot_ms) }, \
         { "gnss_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_navi6d_debug_input_t, gnss_alt) }, \
         { "gnss_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_navi6d_debug_input_t, gnss_speed) }, \
         { "gnss_course", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_navi6d_debug_input_t, gnss_course) }, \
         { "gnss_v", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_navi6d_debug_input_t, gnss_v) }, \
         { "marg_dt", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_navi6d_debug_input_t, marg_dt) }, \
         { "marg_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_navi6d_debug_input_t, marg_acc) }, \
         { "marg_gyr", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_navi6d_debug_input_t, marg_gyr) }, \
         { "marg_mag", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_navi6d_debug_input_t, marg_mag) }, \
         { "baro_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_navi6d_debug_input_t, baro_alt) }, \
         { "odo_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_navi6d_debug_input_t, odo_speed) }, \
         { "gnss_fresh", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_navi6d_debug_input_t, gnss_fresh) }, \
         { "gnss_speed_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 93, offsetof(mavlink_navi6d_debug_input_t, gnss_speed_type) }, \
         { "gnss_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_navi6d_debug_input_t, gnss_fix_type) }, \
         } \
}


/**
 * @brief Pack a navi6d_debug_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param gnss_lat 
 * @param gnss_lon 
 * @param gnss_alt 
 * @param gnss_speed 
 * @param gnss_course 
 * @param gnss_v 3 component speed vector
 * @param gnss_fresh bool flag
 * @param gnss_speed_type 
 * @param gnss_fix_type 
 * @param marg_dt 
 * @param marg_acc 
 * @param marg_gyr 
 * @param marg_mag 
 * @param baro_alt 
 * @param odo_speed 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, double gnss_lat, double gnss_lon, float gnss_alt, float gnss_speed, float gnss_course, const float *gnss_v, uint8_t gnss_fresh, uint8_t gnss_speed_type, uint8_t gnss_fix_type, float marg_dt, const float *marg_acc, const float *marg_gyr, const float *marg_mag, float baro_alt, float odo_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN];
	_mav_put_double(buf, 0, gnss_lat);
	_mav_put_double(buf, 8, gnss_lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, gnss_alt);
	_mav_put_float(buf, 24, gnss_speed);
	_mav_put_float(buf, 28, gnss_course);
	_mav_put_float(buf, 44, marg_dt);
	_mav_put_float(buf, 84, baro_alt);
	_mav_put_float(buf, 88, odo_speed);
	_mav_put_uint8_t(buf, 92, gnss_fresh);
	_mav_put_uint8_t(buf, 93, gnss_speed_type);
	_mav_put_uint8_t(buf, 94, gnss_fix_type);
	_mav_put_float_array(buf, 32, gnss_v, 3);
	_mav_put_float_array(buf, 48, marg_acc, 3);
	_mav_put_float_array(buf, 60, marg_gyr, 3);
	_mav_put_float_array(buf, 72, marg_mag, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#else
	mavlink_navi6d_debug_input_t packet;
	packet.gnss_lat = gnss_lat;
	packet.gnss_lon = gnss_lon;
	packet.time_boot_ms = time_boot_ms;
	packet.gnss_alt = gnss_alt;
	packet.gnss_speed = gnss_speed;
	packet.gnss_course = gnss_course;
	packet.marg_dt = marg_dt;
	packet.baro_alt = baro_alt;
	packet.odo_speed = odo_speed;
	packet.gnss_fresh = gnss_fresh;
	packet.gnss_speed_type = gnss_speed_type;
	packet.gnss_fix_type = gnss_fix_type;
	mav_array_memcpy(packet.gnss_v, gnss_v, sizeof(float)*3);
	mav_array_memcpy(packet.marg_acc, marg_acc, sizeof(float)*3);
	mav_array_memcpy(packet.marg_gyr, marg_gyr, sizeof(float)*3);
	mav_array_memcpy(packet.marg_mag, marg_mag, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
}

/**
 * @brief Pack a navi6d_debug_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param gnss_lat 
 * @param gnss_lon 
 * @param gnss_alt 
 * @param gnss_speed 
 * @param gnss_course 
 * @param gnss_v 3 component speed vector
 * @param gnss_fresh bool flag
 * @param gnss_speed_type 
 * @param gnss_fix_type 
 * @param marg_dt 
 * @param marg_acc 
 * @param marg_gyr 
 * @param marg_mag 
 * @param baro_alt 
 * @param odo_speed 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,double gnss_lat,double gnss_lon,float gnss_alt,float gnss_speed,float gnss_course,const float *gnss_v,uint8_t gnss_fresh,uint8_t gnss_speed_type,uint8_t gnss_fix_type,float marg_dt,const float *marg_acc,const float *marg_gyr,const float *marg_mag,float baro_alt,float odo_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN];
	_mav_put_double(buf, 0, gnss_lat);
	_mav_put_double(buf, 8, gnss_lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, gnss_alt);
	_mav_put_float(buf, 24, gnss_speed);
	_mav_put_float(buf, 28, gnss_course);
	_mav_put_float(buf, 44, marg_dt);
	_mav_put_float(buf, 84, baro_alt);
	_mav_put_float(buf, 88, odo_speed);
	_mav_put_uint8_t(buf, 92, gnss_fresh);
	_mav_put_uint8_t(buf, 93, gnss_speed_type);
	_mav_put_uint8_t(buf, 94, gnss_fix_type);
	_mav_put_float_array(buf, 32, gnss_v, 3);
	_mav_put_float_array(buf, 48, marg_acc, 3);
	_mav_put_float_array(buf, 60, marg_gyr, 3);
	_mav_put_float_array(buf, 72, marg_mag, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#else
	mavlink_navi6d_debug_input_t packet;
	packet.gnss_lat = gnss_lat;
	packet.gnss_lon = gnss_lon;
	packet.time_boot_ms = time_boot_ms;
	packet.gnss_alt = gnss_alt;
	packet.gnss_speed = gnss_speed;
	packet.gnss_course = gnss_course;
	packet.marg_dt = marg_dt;
	packet.baro_alt = baro_alt;
	packet.odo_speed = odo_speed;
	packet.gnss_fresh = gnss_fresh;
	packet.gnss_speed_type = gnss_speed_type;
	packet.gnss_fix_type = gnss_fix_type;
	mav_array_memcpy(packet.gnss_v, gnss_v, sizeof(float)*3);
	mav_array_memcpy(packet.marg_acc, marg_acc, sizeof(float)*3);
	mav_array_memcpy(packet.marg_gyr, marg_gyr, sizeof(float)*3);
	mav_array_memcpy(packet.marg_mag, marg_mag, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
}

/**
 * @brief Encode a navi6d_debug_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param navi6d_debug_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_navi6d_debug_input_t* navi6d_debug_input)
{
	return mavlink_msg_navi6d_debug_input_pack(system_id, component_id, msg, navi6d_debug_input->time_boot_ms, navi6d_debug_input->gnss_lat, navi6d_debug_input->gnss_lon, navi6d_debug_input->gnss_alt, navi6d_debug_input->gnss_speed, navi6d_debug_input->gnss_course, navi6d_debug_input->gnss_v, navi6d_debug_input->gnss_fresh, navi6d_debug_input->gnss_speed_type, navi6d_debug_input->gnss_fix_type, navi6d_debug_input->marg_dt, navi6d_debug_input->marg_acc, navi6d_debug_input->marg_gyr, navi6d_debug_input->marg_mag, navi6d_debug_input->baro_alt, navi6d_debug_input->odo_speed);
}

/**
 * @brief Encode a navi6d_debug_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param navi6d_debug_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_navi6d_debug_input_t* navi6d_debug_input)
{
	return mavlink_msg_navi6d_debug_input_pack_chan(system_id, component_id, chan, msg, navi6d_debug_input->time_boot_ms, navi6d_debug_input->gnss_lat, navi6d_debug_input->gnss_lon, navi6d_debug_input->gnss_alt, navi6d_debug_input->gnss_speed, navi6d_debug_input->gnss_course, navi6d_debug_input->gnss_v, navi6d_debug_input->gnss_fresh, navi6d_debug_input->gnss_speed_type, navi6d_debug_input->gnss_fix_type, navi6d_debug_input->marg_dt, navi6d_debug_input->marg_acc, navi6d_debug_input->marg_gyr, navi6d_debug_input->marg_mag, navi6d_debug_input->baro_alt, navi6d_debug_input->odo_speed);
}

/**
 * @brief Send a navi6d_debug_input message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param gnss_lat 
 * @param gnss_lon 
 * @param gnss_alt 
 * @param gnss_speed 
 * @param gnss_course 
 * @param gnss_v 3 component speed vector
 * @param gnss_fresh bool flag
 * @param gnss_speed_type 
 * @param gnss_fix_type 
 * @param marg_dt 
 * @param marg_acc 
 * @param marg_gyr 
 * @param marg_mag 
 * @param baro_alt 
 * @param odo_speed 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_navi6d_debug_input_send(mavlink_channel_t chan, uint32_t time_boot_ms, double gnss_lat, double gnss_lon, float gnss_alt, float gnss_speed, float gnss_course, const float *gnss_v, uint8_t gnss_fresh, uint8_t gnss_speed_type, uint8_t gnss_fix_type, float marg_dt, const float *marg_acc, const float *marg_gyr, const float *marg_mag, float baro_alt, float odo_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN];
	_mav_put_double(buf, 0, gnss_lat);
	_mav_put_double(buf, 8, gnss_lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, gnss_alt);
	_mav_put_float(buf, 24, gnss_speed);
	_mav_put_float(buf, 28, gnss_course);
	_mav_put_float(buf, 44, marg_dt);
	_mav_put_float(buf, 84, baro_alt);
	_mav_put_float(buf, 88, odo_speed);
	_mav_put_uint8_t(buf, 92, gnss_fresh);
	_mav_put_uint8_t(buf, 93, gnss_speed_type);
	_mav_put_uint8_t(buf, 94, gnss_fix_type);
	_mav_put_float_array(buf, 32, gnss_v, 3);
	_mav_put_float_array(buf, 48, marg_acc, 3);
	_mav_put_float_array(buf, 60, marg_gyr, 3);
	_mav_put_float_array(buf, 72, marg_mag, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
#else
	mavlink_navi6d_debug_input_t packet;
	packet.gnss_lat = gnss_lat;
	packet.gnss_lon = gnss_lon;
	packet.time_boot_ms = time_boot_ms;
	packet.gnss_alt = gnss_alt;
	packet.gnss_speed = gnss_speed;
	packet.gnss_course = gnss_course;
	packet.marg_dt = marg_dt;
	packet.baro_alt = baro_alt;
	packet.odo_speed = odo_speed;
	packet.gnss_fresh = gnss_fresh;
	packet.gnss_speed_type = gnss_speed_type;
	packet.gnss_fix_type = gnss_fix_type;
	mav_array_memcpy(packet.gnss_v, gnss_v, sizeof(float)*3);
	mav_array_memcpy(packet.marg_acc, marg_acc, sizeof(float)*3);
	mav_array_memcpy(packet.marg_gyr, marg_gyr, sizeof(float)*3);
	mav_array_memcpy(packet.marg_mag, marg_mag, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, (const char *)&packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, (const char *)&packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_navi6d_debug_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, double gnss_lat, double gnss_lon, float gnss_alt, float gnss_speed, float gnss_course, const float *gnss_v, uint8_t gnss_fresh, uint8_t gnss_speed_type, uint8_t gnss_fix_type, float marg_dt, const float *marg_acc, const float *marg_gyr, const float *marg_mag, float baro_alt, float odo_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_double(buf, 0, gnss_lat);
	_mav_put_double(buf, 8, gnss_lon);
	_mav_put_uint32_t(buf, 16, time_boot_ms);
	_mav_put_float(buf, 20, gnss_alt);
	_mav_put_float(buf, 24, gnss_speed);
	_mav_put_float(buf, 28, gnss_course);
	_mav_put_float(buf, 44, marg_dt);
	_mav_put_float(buf, 84, baro_alt);
	_mav_put_float(buf, 88, odo_speed);
	_mav_put_uint8_t(buf, 92, gnss_fresh);
	_mav_put_uint8_t(buf, 93, gnss_speed_type);
	_mav_put_uint8_t(buf, 94, gnss_fix_type);
	_mav_put_float_array(buf, 32, gnss_v, 3);
	_mav_put_float_array(buf, 48, marg_acc, 3);
	_mav_put_float_array(buf, 60, marg_gyr, 3);
	_mav_put_float_array(buf, 72, marg_mag, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, buf, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
#else
	mavlink_navi6d_debug_input_t *packet = (mavlink_navi6d_debug_input_t *)msgbuf;
	packet->gnss_lat = gnss_lat;
	packet->gnss_lon = gnss_lon;
	packet->time_boot_ms = time_boot_ms;
	packet->gnss_alt = gnss_alt;
	packet->gnss_speed = gnss_speed;
	packet->gnss_course = gnss_course;
	packet->marg_dt = marg_dt;
	packet->baro_alt = baro_alt;
	packet->odo_speed = odo_speed;
	packet->gnss_fresh = gnss_fresh;
	packet->gnss_speed_type = gnss_speed_type;
	packet->gnss_fix_type = gnss_fix_type;
	mav_array_memcpy(packet->gnss_v, gnss_v, sizeof(float)*3);
	mav_array_memcpy(packet->marg_acc, marg_acc, sizeof(float)*3);
	mav_array_memcpy(packet->marg_gyr, marg_gyr, sizeof(float)*3);
	mav_array_memcpy(packet->marg_mag, marg_mag, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, (const char *)packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT, (const char *)packet, MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NAVI6D_DEBUG_INPUT UNPACKING


/**
 * @brief Get field time_boot_ms from navi6d_debug_input message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_navi6d_debug_input_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field gnss_lat from navi6d_debug_input message
 *
 * @return 
 */
static inline double mavlink_msg_navi6d_debug_input_get_gnss_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field gnss_lon from navi6d_debug_input message
 *
 * @return 
 */
static inline double mavlink_msg_navi6d_debug_input_get_gnss_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field gnss_alt from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_gnss_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gnss_speed from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_gnss_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gnss_course from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_gnss_course(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field gnss_v from navi6d_debug_input message
 *
 * @return 3 component speed vector
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_get_gnss_v(const mavlink_message_t* msg, float *gnss_v)
{
	return _MAV_RETURN_float_array(msg, gnss_v, 3,  32);
}

/**
 * @brief Get field gnss_fresh from navi6d_debug_input message
 *
 * @return bool flag
 */
static inline uint8_t mavlink_msg_navi6d_debug_input_get_gnss_fresh(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  92);
}

/**
 * @brief Get field gnss_speed_type from navi6d_debug_input message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_navi6d_debug_input_get_gnss_speed_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  93);
}

/**
 * @brief Get field gnss_fix_type from navi6d_debug_input message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_navi6d_debug_input_get_gnss_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  94);
}

/**
 * @brief Get field marg_dt from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_marg_dt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field marg_acc from navi6d_debug_input message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_get_marg_acc(const mavlink_message_t* msg, float *marg_acc)
{
	return _MAV_RETURN_float_array(msg, marg_acc, 3,  48);
}

/**
 * @brief Get field marg_gyr from navi6d_debug_input message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_get_marg_gyr(const mavlink_message_t* msg, float *marg_gyr)
{
	return _MAV_RETURN_float_array(msg, marg_gyr, 3,  60);
}

/**
 * @brief Get field marg_mag from navi6d_debug_input message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_navi6d_debug_input_get_marg_mag(const mavlink_message_t* msg, float *marg_mag)
{
	return _MAV_RETURN_float_array(msg, marg_mag, 3,  72);
}

/**
 * @brief Get field baro_alt from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_baro_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field odo_speed from navi6d_debug_input message
 *
 * @return 
 */
static inline float mavlink_msg_navi6d_debug_input_get_odo_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Decode a navi6d_debug_input message into a struct
 *
 * @param msg The message to decode
 * @param navi6d_debug_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_navi6d_debug_input_decode(const mavlink_message_t* msg, mavlink_navi6d_debug_input_t* navi6d_debug_input)
{
#if MAVLINK_NEED_BYTE_SWAP
	navi6d_debug_input->gnss_lat = mavlink_msg_navi6d_debug_input_get_gnss_lat(msg);
	navi6d_debug_input->gnss_lon = mavlink_msg_navi6d_debug_input_get_gnss_lon(msg);
	navi6d_debug_input->time_boot_ms = mavlink_msg_navi6d_debug_input_get_time_boot_ms(msg);
	navi6d_debug_input->gnss_alt = mavlink_msg_navi6d_debug_input_get_gnss_alt(msg);
	navi6d_debug_input->gnss_speed = mavlink_msg_navi6d_debug_input_get_gnss_speed(msg);
	navi6d_debug_input->gnss_course = mavlink_msg_navi6d_debug_input_get_gnss_course(msg);
	mavlink_msg_navi6d_debug_input_get_gnss_v(msg, navi6d_debug_input->gnss_v);
	navi6d_debug_input->marg_dt = mavlink_msg_navi6d_debug_input_get_marg_dt(msg);
	mavlink_msg_navi6d_debug_input_get_marg_acc(msg, navi6d_debug_input->marg_acc);
	mavlink_msg_navi6d_debug_input_get_marg_gyr(msg, navi6d_debug_input->marg_gyr);
	mavlink_msg_navi6d_debug_input_get_marg_mag(msg, navi6d_debug_input->marg_mag);
	navi6d_debug_input->baro_alt = mavlink_msg_navi6d_debug_input_get_baro_alt(msg);
	navi6d_debug_input->odo_speed = mavlink_msg_navi6d_debug_input_get_odo_speed(msg);
	navi6d_debug_input->gnss_fresh = mavlink_msg_navi6d_debug_input_get_gnss_fresh(msg);
	navi6d_debug_input->gnss_speed_type = mavlink_msg_navi6d_debug_input_get_gnss_speed_type(msg);
	navi6d_debug_input->gnss_fix_type = mavlink_msg_navi6d_debug_input_get_gnss_fix_type(msg);
#else
	memcpy(navi6d_debug_input, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT_LEN);
#endif
}
