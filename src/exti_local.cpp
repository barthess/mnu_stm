#include "main.h"
#include "pads.h"

#include "exti_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

ExtiPnc Exti;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
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
static void fpga_err_cb(EXTDriver *extp, expchannel_t channel){
  (void)extp;
  (void)channel;
  orange_led_on();
  osalSysHalt("");
}

/**
 *
 */
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},                                           //0
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},                                           //4
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},                                           //8
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},                                           //12
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOH, fpga_err_cb},
    {EXT_CH_MODE_DISABLED, NULL},                                           //16
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},                                           //20
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
  }
};

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
ExtiPnc::ExtiPnc(void):
ready(false)
{
  return;
}

/**
 *
 */
void ExtiPnc::start(void){
  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, GPIOH_FPGA_IO11);
  ready = true;
}

/**
 *
 */
void ExtiPnc::stop(void){
  extStop(&EXTD1);
  ready = false;
}
