#include "main.h"
#include "pads.h"

#include "msno.hpp"
#include "nmea.hpp"

#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "lapwing/mavlink.h"

using namespace gps;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define HDG_UNKNOWN             65535

#define GPS_DEFAULT_BAUDRATE    115200

#define DEG_TO_MAVLINK          (10 * 1000 * 1000)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/**
 *
 */
static SerialConfig gps_ser_cfg = {
    GPS_DEFAULT_BAUDRATE,
    0,
    0,
    0,
};

static NmeaParser nmea_parser;

static mavlink_message_t rx_msg;
static mavlink_status_t rx_status;

static gps_data_t cache;

static chibios_rt::BinarySemaphore protect_sem(false);

static systime_t last_acquired, prev_acquired;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
static void acquire(void) {
  protect_sem.wait();
}

/**
 *
 */
static void release(void) {
  protect_sem.signal();
}

/**
 *
 */
static void gps_configure(void) {

  gps_ser_cfg.speed = GPS_DEFAULT_BAUDRATE;
  sdStart(&GPSSD, &gps_ser_cfg);
}

/**
 *
 */
static uint8_t buf[256];
static uint8_t idx = 0;
static uint32_t total = 0;
static THD_WORKING_AREA(gpsRxThreadWA, 320);
THD_FUNCTION(msnoRxThread, arg) {
  chRegSetThreadName("gpsRx");
  (void)arg;
  msg_t byte;

  osalThreadSleepSeconds(5);
  gps_configure();

  while (!chThdShouldTerminateX()) {
    byte = sdGetTimeout(&GPSSD, MS2ST(100));
    if (MSG_TIMEOUT != byte) {
      buf[idx] = byte;
      idx++;
      total++;
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
        memset(&rx_msg, 0, sizeof(rx_msg));
      }
    }
  }

  chThdExit(MSG_OK);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void msnoInit(void){

  chThdCreateStatic(gpsRxThreadWA, sizeof(gpsRxThreadWA),
                    NORMALPRIO, msnoRxThread, NULL);
}

/**
 *
 */
void msnoTest(gps_data_t &result) {

  acquire();
  result = cache;
  release();

  osalDbgCheck(ST2S(last_acquired - prev_acquired) < 4);
}
