/** @file
 *	@brief MAVLink comm protocol testsuite generated from lapwing.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef LAPWING_TESTSUITE_H
#define LAPWING_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_lapwing(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_lapwing(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_navi6d_debug_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_navi6d_debug_input_t packet_in = {
		123.0,179.0,963498296,157.0,185.0,213.0,{ 241.0, 242.0, 243.0 },325.0,{ 353.0, 354.0, 355.0 },{ 437.0, 438.0, 439.0 },{ 521.0, 522.0, 523.0 },605.0,633.0,25,92,159
    };
	mavlink_navi6d_debug_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gnss_lat = packet_in.gnss_lat;
        	packet1.gnss_lon = packet_in.gnss_lon;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.gnss_alt = packet_in.gnss_alt;
        	packet1.gnss_speed = packet_in.gnss_speed;
        	packet1.gnss_course = packet_in.gnss_course;
        	packet1.marg_dt = packet_in.marg_dt;
        	packet1.baro_alt = packet_in.baro_alt;
        	packet1.odo_speed = packet_in.odo_speed;
        	packet1.gnss_fresh = packet_in.gnss_fresh;
        	packet1.gnss_speed_type = packet_in.gnss_speed_type;
        	packet1.gnss_fix_type = packet_in.gnss_fix_type;
        
        	mav_array_memcpy(packet1.gnss_v, packet_in.gnss_v, sizeof(float)*3);
        	mav_array_memcpy(packet1.marg_acc, packet_in.marg_acc, sizeof(float)*3);
        	mav_array_memcpy(packet1.marg_gyr, packet_in.marg_gyr, sizeof(float)*3);
        	mav_array_memcpy(packet1.marg_mag, packet_in.marg_mag, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_input_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_navi6d_debug_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_input_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.gnss_lat , packet1.gnss_lon , packet1.gnss_alt , packet1.gnss_speed , packet1.gnss_course , packet1.gnss_v , packet1.gnss_fresh , packet1.gnss_speed_type , packet1.gnss_fix_type , packet1.marg_dt , packet1.marg_acc , packet1.marg_gyr , packet1.marg_mag , packet1.baro_alt , packet1.odo_speed );
	mavlink_msg_navi6d_debug_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.gnss_lat , packet1.gnss_lon , packet1.gnss_alt , packet1.gnss_speed , packet1.gnss_course , packet1.gnss_v , packet1.gnss_fresh , packet1.gnss_speed_type , packet1.gnss_fix_type , packet1.marg_dt , packet1.marg_acc , packet1.marg_gyr , packet1.marg_mag , packet1.baro_alt , packet1.odo_speed );
	mavlink_msg_navi6d_debug_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_navi6d_debug_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_input_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.gnss_lat , packet1.gnss_lon , packet1.gnss_alt , packet1.gnss_speed , packet1.gnss_course , packet1.gnss_v , packet1.gnss_fresh , packet1.gnss_speed_type , packet1.gnss_fix_type , packet1.marg_dt , packet1.marg_acc , packet1.marg_gyr , packet1.marg_mag , packet1.baro_alt , packet1.odo_speed );
	mavlink_msg_navi6d_debug_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_navi6d_debug_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_navi6d_debug_output_t packet_in = {
		123.0,179.0,963498296,157.0,185.0,213.0,241.0,113,180
    };
	mavlink_navi6d_debug_output_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.alt = packet_in.alt;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.kalman_state_size = packet_in.kalman_state_size;
        	packet1.kalman_meas_size = packet_in.kalman_meas_size;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_output_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_navi6d_debug_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_output_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.kalman_state_size , packet1.kalman_meas_size );
	mavlink_msg_navi6d_debug_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_output_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.kalman_state_size , packet1.kalman_meas_size );
	mavlink_msg_navi6d_debug_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_navi6d_debug_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navi6d_debug_output_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.kalman_state_size , packet1.kalman_meas_size );
	mavlink_msg_navi6d_debug_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lapwing(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_navi6d_debug_input(system_id, component_id, last_msg);
	mavlink_test_navi6d_debug_output(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LAPWING_TESTSUITE_H
