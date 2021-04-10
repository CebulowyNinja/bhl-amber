/**
 * @file max30100.c
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * @brief This is the source code for all the functions included in the
 * MAX30100 ESP32 Library.
*/

#include <stdio.h>
#include "max30100/max30100.h"
#include "max30100/registers.h"
#include <string.h>

esp_err_t max30100_init( max30100_config_t* this,
                         i2c_port_t i2c_num,
                         max30100_mode_t mode,
                         max30100_sampling_rate_t sampling_rate,
                         max30100_pulse_width_t pulse_width,
                         max30100_current_t ir_current,
                         max30100_current_t red_current,
                         uint16_t buffer_size,
                         max30100_adc_range_t adc_range,
                         bool debug )
{
    this->i2c_num = i2c_num;
    this->debug = debug;
    esp_err_t ret = max30100_set_mode(this, mode);
    if(ret != ESP_OK) return ret;
    
    ret = max330100_set_slots(this, MAX30100_SLOT_RED, MAX30100_SLOT_IR, MAX30100_SLOT_NONE, MAX30100_SLOT_NONE);
    if(ret != ESP_OK) return ret;
    
    ret = max30100_set_sampling_rate(this, sampling_rate);
    if(ret != ESP_OK) return ret;
    ret = max30100_set_pulse_width(this, pulse_width);
    if(ret != ESP_OK) return ret;
  
    this->red_current = red_current;
    this->ir_current = ir_current;
    ret = max30100_set_led_current(this, this->red_current, this->ir_current);
    if(ret != ESP_OK) return ret;

    ret = max30100_set_adc_range(this, adc_range);
    if(ret != ESP_OK) return ret;

    this->buffer_size = buffer_size;
    this->buffer.data = malloc(sizeof(uint16_t)*this->buffer_size);

    if(this->buffer.data == NULL) {

        return ESP_ERR_INVALID_RESPONSE;
    }
    this->buffer.count = 0;
    this->buffer.index = 0;
    this->probes_num = 0;

    return ESP_OK;
}

float max30100_calc_avg(max30100_config_t* this);
float max30100_calc_var(max30100_config_t* this, float avg);
float max30100_largest_autocorrelation(max30100_config_t* this, float avg, uint16_t min_diff, uint16_t max_diff);
esp_err_t max30100_update(max30100_config_t* this, max30100_data_t* data) {
    data->pulse_detected = false;
    data->heart_bpm = 0.0;

    max30100_fifo_t raw_data;
    esp_err_t ret = max30100_read_fifo(this, &raw_data);

    if(ret != ESP_OK) return ret;
    uint16_t val = raw_data.raw_ir;
    this->buffer.data[this->buffer.index] = val;
    this->buffer.index++;
    this->buffer.index %= this->buffer_size;
    if(this->buffer.count < this->buffer_size) {
        this->buffer.count++;
    }
    data->heart_bpm = this->current_bpm;
    this->probes_num++;
    if(this->probes_num % 600 == 0) {
        float avg = max30100_calc_avg(this);
        float var = max30100_calc_var(this, avg);
        printf("avg = %f, var=%f\n", avg, var);
        float corr_diff = max30100_largest_autocorrelation(this, avg, MIN_CORR_DIFF, MAX_CORR_DIFF);
        printf("corr_diff=%f, larg_autocorr = %f\n", corr_diff, 100/corr_diff*60);
        for(uint16_t i = 0; i < this->buffer.count; i++) {
            uint16_t ind = (this->buffer.index + i) % this->buffer.count;
            printf("%d\n", this->buffer.data[ind]);
        }
        this->probes_num = 1;
    }
    return ESP_OK;
}

float max30100_calc_avg(max30100_config_t* this) {
    float sum = 0;
    for(uint16_t i = 0; i < this->buffer.count; i++) {
        uint16_t ind = (this->buffer.index + i) % this->buffer.count;
        sum += (float) this->buffer.data[ind];
    }
    return sum/this->buffer.count;
}
float max30100_calc_var(max30100_config_t* this, float avg) {
    float sum = 0;
    for(uint16_t i = 0; i < this->buffer.count; i++) {
        uint16_t ind = (this->buffer.index + i) % this->buffer.count;
        float diff = ((float) this->buffer.data[ind] - avg);
        sum += diff*diff;
    }
    return sqrt(sum/this->buffer.count);
}
float max30100_largest_autocorrelation(max30100_config_t* this, float avg, uint16_t min_diff, uint16_t max_diff) {
    float max_corr = 0;
    uint16_t max_corr_diff = min_diff;
    uint16_t* data = this->buffer.data;
    for(uint16_t diff = min_diff; diff <= max_diff; diff++) {
        float corr = 0;
        for(uint16_t j = 0; j < this->buffer.count; j++) {
            uint16_t ind = (j + this->buffer.index)%this->buffer.count;
            uint16_t ind2 = (ind + diff)%this->buffer.count;
            corr += ((float) data[ind] - avg)*((float) data[ind2] - avg);
        }
        //printf("%d, %f\n", diff, corr);
        if(corr > max_corr) {
            max_corr_diff = diff;
            max_corr = corr;
        }
    }
    return max_corr_diff;
}
// Writes val to address register on device
esp_err_t max30100_write_register( max30100_config_t* this,
                                   uint8_t address,
                                   uint8_t val      )
{
    // start transmission to device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_DEVICE << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, address, true); // send register address
    i2c_master_write_byte(cmd, val, true); // send value to write

    // end transmission
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t max30100_read_register( max30100_config_t* this,
                                  uint8_t address,
                                  uint8_t* reg     )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_DEVICE << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);

    //i2c_master_stop(cmd);
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (MAX30100_DEVICE << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, reg, 1); //1 is NACK

    // end transmission
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );

    i2c_cmd_link_delete(cmd);
    return ret;
}

// Reads num bytes starting from address register on device in to _buff array
esp_err_t max30100_read_from( max30100_config_t* this,
                              uint8_t address,
                              uint8_t* reg,
                              uint8_t size )
{
    if(!size)
        return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_DEVICE << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, address, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30100_DEVICE << 1) | I2C_MASTER_READ, true);

    if(size > 1)
        i2c_master_read(cmd, reg, size-1, 0); //0 is ACK

    i2c_master_read_byte(cmd, reg+size-1, 1); //1 is NACK

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin( this->i2c_num,
                                          cmd,
                                          1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t max30100_set_mode(max30100_config_t* this, max30100_mode_t mode) {
    uint8_t current_mode_reg;
    //Tratar erros
    esp_err_t ret = max30100_read_register( this,
                                            MAX30100_MODE_CONF,
                                            &current_mode_reg );
    if(ret != ESP_OK) return ret;
    return max30100_write_register( this,
                                    MAX30100_MODE_CONF,
                                    (current_mode_reg & 0xF8) | mode );
}

esp_err_t max30100_set_adc_range(max30100_config_t* this, max30100_adc_range_t adc_range) {
    uint8_t previous;

    //Tratar erros
    esp_err_t ret = max30100_read_register(this, MAX30100_SPO2_CONF, &previous);
    if(ret != ESP_OK) return ret;
    return max30100_write_register( this,
                                    MAX30100_SPO2_CONF,
                                    ((previous & 0x60) | (adc_range << 5)));
}

esp_err_t max30100_set_sampling_rate( max30100_config_t* this,
                                      max30100_sampling_rate_t rate )
{
    uint8_t current_spO2_reg;

    //Tratar erros
    esp_err_t ret = max30100_read_register( this,
                                            MAX30100_SPO2_CONF,
                                            &current_spO2_reg );
    if(ret != ESP_OK) return ret;
    return max30100_write_register( this,
                                    MAX30100_SPO2_CONF,
                                    (current_spO2_reg & 0xE3) | (rate<<2) );
}

esp_err_t max30100_set_pulse_width( max30100_config_t* this,
                                    max30100_pulse_width_t pw )
{
    uint8_t current_spO2_reg;

    //Tratar erros
    esp_err_t ret = max30100_read_register( this,
                                            MAX30100_SPO2_CONF,
                                            &current_spO2_reg );
    if(ret != ESP_OK) return ret;
    return max30100_write_register( this,
                                    MAX30100_SPO2_CONF,
                                    (current_spO2_reg & 0xFC) | pw );
}

esp_err_t max30100_set_led_current( max30100_config_t* this,
                                    max30100_current_t red_current,
                                    max30100_current_t ir_current )
{
    //Tratar erros
    esp_err_t ret = max30100_write_register( this,
                                    MAX30100_LED1_CONF,
                                    red_current);
    if(ret != ESP_OK) return ret;
    return max30100_write_register( this,
                                    MAX30100_LED2_CONF,
                                    ir_current);
}

esp_err_t max330100_set_slots(max30100_config_t* this,  
    uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4) {
    esp_err_t ret =  max30100_write_register(this,
                                        MAX30100_SLOT_21,
                                        (slot2<<4) | (slot1));
    if(ret != ESP_OK) return ret;

    return max30100_write_register(this,
                                   MAX30100_SLOT_43,
                                   (slot4<<4) | (slot3));
}

esp_err_t max330100_start_read_temperature(max30100_config_t* this) {
    return max30100_write_register(this,
                                   MAX30100_TEMP_CONFIG,
                                   MAX30100_MODE_TEMP_EN);
}
esp_err_t max330100_read_temperature(max30100_config_t* this, float* temperature) {
    esp_err_t ret = max330100_start_read_temperature(this);
    if(ret != ESP_OK) return ret;

    //This can be changed to a while loop, (with interrupt flag!)
    //there is an interrupt flag for when temperature has been read.
    vTaskDelay(100/portTICK_PERIOD_MS);

    int8_t temp;
    //Tratar erros
    ret = max30100_read_register(this, MAX30100_TEMP_INT, (uint8_t*)&temp);
    if(ret != ESP_OK) return ret;

    float temp_fraction;
    ret = max30100_read_register( this,
                                  MAX30100_TEMP_FRACTION,
                                  (uint8_t*)&temp_fraction);
    if(ret != ESP_OK) return ret;
    temp_fraction *= 0.0625;
    *temperature = (float)temp+temp_fraction;
    return ESP_OK;
}

esp_err_t max30100_read_fifo(max30100_config_t* this, max30100_fifo_t* fifo) {
    uint8_t buffer[4];
    //Testar erros
    esp_err_t ret = max30100_read_from(this, MAX30100_FIFO_DATA, buffer, 6);
    if(ret != ESP_OK) return ret;
    uint32_t tmp = ((uint32_t)buffer[0] << 16) | ((uint32_t) buffer[1] << 8) | buffer[2];
    fifo->raw_red = (uint16_t) ((tmp & 0x3FFFF) >> 2);
    tmp = ((uint32_t)buffer[3] << 16) | ((uint32_t) buffer[4] << 8) | buffer[5];
    fifo->raw_ir = (uint16_t) ((tmp & 0x3FFFF) >> 2);

    return ESP_OK;
}

esp_err_t max30100_print_registers(max30100_config_t* this)
{
    uint8_t int_status, int_enable, fifo_write, fifo_ovf_cnt, fifo_read;
    uint8_t fifo_data, mode_conf, sp02_conf, led1_conf, led2_conf, slot21_conf, slot43_conf, temp_int, temp_frac;
    uint8_t rev_id, part_id;
    esp_err_t ret;

    ret = max30100_read_register(this, MAX30100_INT_STATUS, &int_status);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_INT_ENABLE, &int_enable);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_FIFO_WRITE, &fifo_write);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register( this,
                                  MAX30100_FIFO_OVERFLOW_COUNTER,
                                  &fifo_ovf_cnt );
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_FIFO_READ, &fifo_read);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_FIFO_DATA, &fifo_data);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_MODE_CONF, &mode_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_SPO2_CONF, &sp02_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_LED1_CONF, &led1_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_LED1_CONF, &led2_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_SLOT_21, &slot21_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_SLOT_43, &slot43_conf);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_TEMP_INT, &temp_int);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_TEMP_FRACTION, &temp_frac);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_REV_ID, &rev_id);
    if(ret != ESP_OK) return ret;
    ret = max30100_read_register(this, MAX30100_PART_ID, &part_id);
    if(ret != ESP_OK) return ret;

    printf("%x\n", int_status);
    printf("%x\n", int_enable);
    printf("%x\n", fifo_write);
    printf("%x\n", fifo_ovf_cnt);
    printf("%x\n", fifo_read);
    printf("%x\n", fifo_data);
    printf("MODE=%x\n", mode_conf);
    printf("SP02=%x\n", sp02_conf);
    printf("LED1=%x\n", led1_conf);
    printf("LED2=%x\n", led2_conf);
    printf("SLOT21=%x\n", slot21_conf);
    printf("SLOT43=%x\n", slot43_conf);
    printf("TEMP_INT=%x\n", temp_int);
    printf("TEMP_FRAC=%x\n", temp_frac);
    printf("REV=%x\n", rev_id);
    printf("PART=%x\n", part_id);

    return ESP_OK;
}

/**
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/