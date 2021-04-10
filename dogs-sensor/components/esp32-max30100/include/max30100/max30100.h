/**
 * @file max30100.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * @brief This library was created to interface the MAX30100 pulse and oxymeter
 * sensor with ESP32 using the IDF-SDK. It includes functions to initialize with
 * the programmer's desired parameters and to update the readings, detecting pulse
 * and having the pulse saturation O2. It is based on Strogonovs Arduino library.
*/
#ifndef MAX30100_H
#define MAX30100_H

#include <math.h>
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_err.h"

/**
 * Default parameters for initialization.
 */
#define MAX30100_DEFAULT_OPERATING_MODE MAX30100_MODE_SPO2_HR
#define MAX30100_DEFAULT_IR_LED_CURRENT MAX30100_LED_CURRENT_50MA
#define MAX30100_DEFAULT_START_RED_LED_CURRENT MAX30100_LED_CURRENT_25_4MA
#define MAX30100_DEFAULT_SAMPLING_RATE MAX30100_SAMPLING_RATE_100HZ
#define MAX30100_DEFAULT_LED_PULSE_WIDTH MAX30100_PULSE_WIDTH_411US_ADC_18
#define MAX30100_DEFAULT_ACCEPTABLE_INTENSITY_DIFF 65000
#define MAX30100_DEFAULT_RED_LED_CURRENT_ADJUSTMENT_MS 500
#define MAX30100_DEFAULT_RESET_SPO2_EVERY_N_PULSES 4
#define MAX30100_DEFAULT_ALPHA 0.95
#define MAX30100_DEFAULT_MEAN_FILTER_SIZE 15
#define MAX30100_DEFAULT_PULSE_MIN_THRESHOLD 300
#define MAX30100_DEFAULT_PULSE_MAX_THRESHOLD 2000
#define MAX30100_DEFAULT_PULSE_BPM_SAMPLE_SIZE 10

#define BUFFER_COUNT 600
#define MIN_CORR_DIFF 30 
#define MAX_CORR_DIFF 150

typedef enum _max30100_slot_t
{
    MAX30100_SLOT_NONE = 0x00,
    MAX30100_SLOT_RED = 0x01,
    MAX30100_SLOT_IR = 0x02
} _max30100_slot_t;

typedef enum _max30100_mode_t
{
    MAX30100_MODE_HR_ONLY = 0x02,
    MAX30100_MODE_SPO2_HR = 0x03,
    MAX30100_MODE_MULTI = 0x07
} max30100_mode_t;

typedef enum SamplingRate
{
    MAX30100_SAMPLING_RATE_50HZ = 0x00,
    MAX30100_SAMPLING_RATE_100HZ = 0x01,
    MAX30100_SAMPLING_RATE_200HZ = 0x02,
    MAX30100_SAMPLING_RATE_400HZ = 0x03,
    MAX30100_SAMPLING_RATE_800HZ = 0x04,
    MAX30100_SAMPLING_RATE_1000HZ = 0x05,
    MAX30100_SAMPLING_RATE_1600HZ = 0x06,
    MAX30100_SAMPLING_RATE_3200HZ = 0x07
} max30100_sampling_rate_t;

typedef enum LEDPulseWidth
{
    MAX30100_PULSE_WIDTH_69US_ADC_15 = 0x00,
    MAX30100_PULSE_WIDTH_118US_ADC_16 = 0x01,
    MAX30100_PULSE_WIDTH_215US_ADC_17 = 0x02,
    MAX30100_PULSE_WIDTH_411US_ADC_18 = 0x03
} max30100_pulse_width_t;

typedef enum LEDCurrent
{
    MAX30100_LED_CURRENT_0MA = 0x00,
    MAX30100_LED_CURRENT_25_4MA = 0x7F,
    MAX30100_LED_CURRENT_50MA = 0xFF
} max30100_current_t;

typedef enum ADCRange
{
    MAX30100_ADC_RANGE_2048nA = 0x00,
    MAX30100_ADC_RANGE_4096nA = 0x01,
    MAX30100_ADC_RANGE_8192nA = 0x02,
    MAX30100_ADC_RANGE_16384nA = 0x03,
} max30100_adc_range_t;

typedef struct _max30100_fifo_t
{
    uint16_t raw_ir;
    uint16_t raw_red;
} max30100_fifo_t;

typedef struct _round_buffer_t
{
    uint16_t *data;
    uint16_t count;
    uint16_t index;
} round_buffer_t;

typedef struct _max30100_config_t
{
    i2c_port_t i2c_num;
    max30100_current_t red_current;
    max30100_current_t ir_current;

    bool debug;
    uint16_t buffer_size;
    float current_bpm;
    round_buffer_t buffer;
    uint16_t probes_num;
} max30100_config_t;

typedef struct _max30100_data_t
{
    bool pulse_detected;
    float heart_bpm;
} max30100_data_t;

esp_err_t max30100_init(max30100_config_t *this, i2c_port_t i2c_num,
              max30100_mode_t mode, max30100_sampling_rate_t sampling_rate,
              max30100_pulse_width_t pulse_width, max30100_current_t ir_current,
              max30100_current_t red_current, uint16_t buffer_size,
              max30100_adc_range_t adc_range, bool debug);

esp_err_t max30100_update(max30100_config_t *this, max30100_data_t *data);

esp_err_t max330100_set_slots(max30100_config_t *this,
                              uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4);

esp_err_t max330100_start_read_temperature(max30100_config_t *this);

esp_err_t max30100_read_temperature(max30100_config_t *this, float *temperature);

esp_err_t max30100_print_registers(max30100_config_t *this);

esp_err_t max30100_set_mode(max30100_config_t *this, max30100_mode_t mode);

esp_err_t max30100_set_adc_range(max30100_config_t *this, max30100_adc_range_t adc_range);

esp_err_t max30100_set_led_current(max30100_config_t *this,
                                   max30100_current_t red_current,
                                   max30100_current_t ir_current);

esp_err_t max30100_set_pulse_width(max30100_config_t *this, max30100_pulse_width_t pw);

esp_err_t max30100_set_sampling_rate(max30100_config_t *this,
                                     max30100_sampling_rate_t rate);

#endif
