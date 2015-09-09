#include "main.h"
#include "alpha_beta.hpp"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ADC_NUM_CHANNELS          2
#define ADC_BUF_DEPTH             1


/* human readable names */
#define ADC_BOARD_VOLTAGE_P       ADC_CHANNEL_IN10
#define ADC_BOARD_VOLTAGE_N       ADC_CHANNEL_IN11

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void adc_cb(ADCDriver *adcp, adcsample_t *samples, size_t n);
static void adc_eeror_cb(ADCDriver *adcp, adcerror_t err);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static size_t errors = 0;

static ADCConfig adccfg; // dummy for STM32

static adcsample_t samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

/*
 * ADC conversion group.
 */
static const ADCConversionGroup adccg = {
  TRUE,
  ADC_NUM_CHANNELS,
  NULL,//adc_cb,
  adc_eeror_cb,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480)    |
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ2_N(ADC_BOARD_VOLTAGE_N)    |
  ADC_SQR3_SQ1_N(ADC_BOARD_VOLTAGE_P)
};

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/*
 * ADC streaming callback.
 */
static void adc_cb(ADCDriver *adcp, adcsample_t *samples, size_t n) {
  (void)adcp;
  (void)samples;
  (void)n;

  //temp_filter(samples[ADC_MPXV_TEMP_CH - CHANNEL_OFFSET]);
}

/*
 *
 */
static void adc_eeror_cb(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  errors++;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void ADCInitLocal(void) {
  (void)adc_cb;

  adcStart(&ADCD1, &adccfg);
  adcStartConversion(&ADCD1, &adccg, samples, ADC_BUF_DEPTH);
}

/**
 * @brief   Return millivolts.
 */
uint32_t ADCgetBoardVoltage(void) {
  uint32_t diff;
  uint32_t millivolts;

  diff  = samples[0] - samples[1];
  millivolts = (diff * 10000) / 568;

  osalDbgCheck((millivolts > 6000) && (millivolts < 50000));

  return millivolts;
}


