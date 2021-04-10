/**
 * @file registers.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * @brief This is the "private" headers file for the MAX30100 ESP32 library.
 * Please do NOT include in your code.
*/

#ifndef MAX30100_REGISTERS_H
#define MAX30100_REGISTERS_H

#include "max30100.h"

/**
 * MAX30100 internal registers definitions.
 */
#define MAX30100_DEVICE                   0x57
#define MAX30100_REV_ID                   0xFE
#define MAX30100_PART_ID                  0xFF
#define MAX30100_INT_STATUS               0x00
#define MAX30100_INT_ENABLE               0x02
#define MAX30100_FIFO_WRITE               0x04
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x05
#define MAX30100_FIFO_READ                0x06
#define MAX30100_FIFO_DATA                0x07
#define MAX30100_MODE_CONF                0x09
#define MAX30100_SPO2_CONF                0x0A
#define MAX30100_LED1_CONF                0x0C
#define MAX30100_LED2_CONF                0x0D
#define MAX30100_SLOT_21                  0x11
#define MAX30100_SLOT_43                  0x12
#define MAX30100_TEMP_INT                 0x1F
#define MAX30100_TEMP_FRACTION            0x20
#define MAX30100_TEMP_CONFIG              0x21

/**
 * Bit defines for mode configuration.
 */
#define MAX30100_MODE_SHDN                (1<<7)
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<0)
#define MAX30100_SPO2_HI_RES_EN           (1<<6)

/**
 * Pulse state machine Enum.
 */
typedef enum _pulse_state_machine {
    MAX30100_PULSE_IDLE,
    MAX30100_PULSE_TRACE_UP,
    MAX30100_PULSE_TRACE_DOWN
} pulse_state_machine;

/**
 * "Private" functions declarations.
 * These functions will only be called by the library, and not by the user.
 */

/**
 * @brief Write to the MAX30100 register.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param val is the byte to write.
 * 
 * @returns status of execution.
 */
esp_err_t max30100_write_register(max30100_config_t* this, uint8_t address, uint8_t val);

/**
 * @brief Read from MAX30100 register.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param reg is the address to save the byte read.
 * 
 * @returns status of execution.
 */
esp_err_t max30100_read_register(max30100_config_t* this, uint8_t address, uint8_t* reg);

/**
 * @brief Read set of MAX30100 registers.
 * 
 * @param this is the address of the configuration structure.
 * @param address is the register address.
 * @param data is the initial address to save.
 * @param size is the size of the register to read.
 * 
 * @returns status of execution.
 */
esp_err_t max30100_read_from( max30100_config_t* this, uint8_t address, 
                              uint8_t* data, uint8_t size );

/**
 * @brief Read from MAX30100 FIFO.
 * 
 * @param this is the address of the configuration structure.
 * @param fifo is the address to a FIFO structure.
 * 
 * @returns status of execution.
 */
esp_err_t max30100_read_fifo(max30100_config_t* this, max30100_fifo_t* fifo);

#endif

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