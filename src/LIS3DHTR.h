/*
    The MIT License (MIT)

    Author: Hongtai Liu (lht856@foxmail.com)

    Copyright (C) 2019  Seeed Technology Co.,Ltd.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <SPI.h>

#ifndef SEEED_LIS3DHTR_H
#define SEEED_LIS3DHTR_H

/**************************************************************************
    I2C ADDRESS/BITS
**************************************************************************/
#define LIS3DHTR_DEFAULT_ADDRESS (0x18) // 3C >> 1 = 7-bit default
#define LIS3DHTR_ADDRESS_UPDATED (0x19) //

/**************************************************************************
    CONVERSION DELAY (in mS)
**************************************************************************/
#define LIS3DHTR_CONVERSIONDELAY (100)

/**************************************************************************
    ACCELEROMETER REGISTERS
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_STATUS (0x07)        // Status Register
#define LIS3DHTR_REG_OUT_ADC1_L (0x08)          // Auxiliary ADC Channel 1 Low Register
#define LIS3DHTR_REG_OUT_ADC1_H (0x09)          // Auxiliary ADC Channel 1 High Register
#define LIS3DHTR_REG_OUT_ADC2_L (0x0A)          // Auxiliary ADC Channel 2 Low Register
#define LIS3DHTR_REG_OUT_ADC2_H (0x0B)          // Auxiliary ADC Channel 2 High Register
#define LIS3DHTR_REG_OUT_ADC3_L (0x0C)          // Auxiliary ADC Channel 3 & Temperature Low Register
#define LIS3DHTR_REG_OUT_ADC3_H (0x0D)          // Auxiliary ADC Channel 3 & Temperature High Register
#define LIS3DHTR_REG_ACCEL_WHO_AM_I (0x0F)      // Device identification Register
#define LIS3DHTR_REG_TEMP_CFG (0x1F)            // Temperature Sensor Register
#define LIS3DHTR_REG_ACCEL_CTRL_REG1 (0x20)     // Accelerometer Control Register 1
#define LIS3DHTR_REG_ACCEL_CTRL_REG2 (0x21)     // Accelerometer Control Register 2
#define LIS3DHTR_REG_ACCEL_CTRL_REG3 (0x22)     // Accelerometer Control Register 3
#define LIS3DHTR_REG_ACCEL_CTRL_REG4 (0x23)     // Accelerometer Control Register 4
#define LIS3DHTR_REG_ACCEL_CTRL_REG5 (0x24)     // Accelerometer Control Register 5
#define LIS3DHTR_REG_ACCEL_CTRL_REG6 (0x25)     // Accelerometer Control Register 6
#define LIS3DHTR_REG_ACCEL_REFERENCE (0x26)     // Reference/Datacapture Register
#define LIS3DHTR_REG_ACCEL_STATUS2 (0x27)       // Status Register 2
#define LIS3DHTR_REG_ACCEL_OUT_X_L (0x28)       // X-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_X_H (0x29)       // X-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_Y_L (0x2A)       // Y-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_Y_H (0x2B)       // Y-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_OUT_Z_L (0x2C)       // Z-Axis Acceleration Data Low Register
#define LIS3DHTR_REG_ACCEL_OUT_Z_H (0x2D)       // Z-Axis Acceleration Data High Register
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL (0x2E)     // FIFO Control Register
#define LIS3DHTR_REG_ACCEL_FIFO_SRC (0x2F)      // FIFO Source Register
#define LIS3DHTR_REG_ACCEL_INT1_CFG (0x30)      // Interrupt Configuration Register
#define LIS3DHTR_REG_ACCEL_INT1_SRC (0x31)      // Interrupt Source Register
#define LIS3DHTR_REG_ACCEL_INT1_THS (0x32)      // Interrupt Threshold Register
#define LIS3DHTR_REG_ACCEL_INT1_DURATION (0x33) // Interrupt Duration Register
#define LIS3DHTR_REG_ACCEL_CLICK_CFG (0x38)     // Interrupt Click Recognition Register
#define LIS3DHTR_REG_ACCEL_CLICK_SRC (0x39)     // Interrupt Click Source Register
#define LIS3DHTR_REG_ACCEL_CLICK_THS (0x3A)     // Interrupt Click Threshold Register
#define LIS3DHTR_REG_ACCEL_TIME_LIMIT (0x3B)    // Click Time Limit Register
#define LIS3DHTR_REG_ACCEL_TIME_LATENCY (0x3C)  // Click Time Latency Register
#define LIS3DHTR_REG_ACCEL_TIME_WINDOW (0x3D)   // Click Time Window Register

/**************************************************************************
    TEMPERATURE REGISTER DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_TEMP_ADC_PD_MASK (0x80)     // ADC Power Enable Status
#define LIS3DHTR_REG_TEMP_ADC_PD_DISABLED (0x00) // ADC Disabled
#define LIS3DHTR_REG_TEMP_ADC_PD_ENABLED (0x80)  // ADC Enabled

#define LIS3DHTR_REG_TEMP_TEMP_EN_MASK (0x40)     // Temperature Sensor (T) Enable Status
#define LIS3DHTR_REG_TEMP_TEMP_EN_DISABLED (0x00) // Temperature Sensor (T) Disabled
#define LIS3DHTR_REG_TEMP_TEMP_EN_ENABLED (0x40)  // Temperature Sensor (T) Enabled

/**************************************************************************
    ACCELEROMETER CONTROL REGISTER 1 DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_MASK (0xF0) // Acceleration Data Rate Selection
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_PD (0x00)   // Power-Down Mode
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1 (0x10)    // Normal / Low Power Mode (1 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_10 (0x20)   // Normal / Low Power Mode (10 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_25 (0x30)   // Normal / Low Power Mode (25 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_50 (0x40)   // Normal / Low Power Mode (50 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_100 (0x50)  // Normal / Low Power Mode (100 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_200 (0x60)  // Normal / Low Power Mode (200 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_400 (0x70)  // Normal / Low Power Mode (400 Hz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1_6K (0x80) // Low Power Mode (1.6 KHz)
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_5K (0x90)   // Normal (1.25 KHz) / Low Power Mode (5 KHz)

#define LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_MASK (0x08)   // Low Power Mode Enable
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL (0x00) // Normal Mode
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_LOW (0x08)    // Low Power Mode

#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_MASK (0x04)    // Acceleration Z-Axis Enable
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_DISABLE (0x00) // Acceleration Z-Axis Disabled
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE (0x04)  // Acceleration Z-Axis Enabled

#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_MASK (0x02)    // Acceleration Y-Axis Enable
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_DISABLE (0x00) // Acceleration Y-Axis Disabled
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE (0x02)  // Acceleration Y-Axis Enabled

#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_MASK (0x01)    // Acceleration X-Axis Enable
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_DISABLE (0x00) // Acceleration X-Axis Disabled
#define LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE (0x01)  // Acceleration X-Axis Enabled

/**************************************************************************
    ACCELEROMETER CONTROL REGISTER 2 DESCRIPTION
**************************************************************************/
#define LIS3DHTR_CTRL_REG2_HPM_MASK (0xC0)      // HPM
#define LIS3DHTR_CTRL_REG2_HPM_NORMAL (0x00)    // Normal mode
#define LIS3DHTR_CTRL_REG2_HPM_REF (0x40)       // Reference signal for filtering
#define LIS3DHTR_CTRL_REG2_HPM_NORMAL1 (0x80)   // Normal mode
#define LIS3DHTR_CTRL_REG2_HPM_AUTORESET (0xC0) // Autoreset on interrupt event

#define LIS3DHTR_CTRL_REG2_HPCF_MASK (0x30) // HPCF
#define LIS3DHTR_CTRL_REG2_HPCF_NONE (0x00) // NONE

#define LIS3DHTR_CTRL_REG2_FDS_MASK (0x08)    // Filtered data selection
#define LIS3DHTR_CTRL_REG2_FDS_DISABLE (0x00) // filter disable
#define LIS3DHTR_CTRL_REG2_FDS_ENABLE (0x08)  // filter enable

#define LIS3DHTR_CTRL_REG2_HPCLICK_MASK (0x04)    // High-pass filter enabled for CLICK function.
#define LIS3DHTR_CTRL_REG2_HPCLICK_DISABLE (0x00) // filter disable
#define LIS3DHTR_CTRL_REG2_HPCLICK_ENABLE (0x04)  // filter enable

#define LIS3DHTR_CTRL_REG2_HP_IA2_MASK (0x02)    // High-pass filter enabled for AOI function on interrupt 2
#define LIS3DHTR_CTRL_REG2_HP_IA2_DISABLE (0x00) // filter disable
#define LIS3DHTR_CTRL_REG2_HP_IA2_ENABLE (0x02)

#define LIS3DHTR_CTRL_REG2_HP_IA1_MASK (0x01)    // High-pass filter enabled for AOI function on interrupt 1
#define LIS3DHTR_CTRL_REG2_HP_IA1_DISABLE (0x00) // filter disable
#define LIS3DHTR_CTRL_REG2_HP_IA1_ENABLE (0x01)  // filter enable

/**************************************************************************
    ACCELEROMETER CONTROL REGISTER 3 DESCRIPTION
**************************************************************************/
#define LIS3DHTR_CTRL_REG3_CLICK_MASK (0x80)    // Click interrupt on INT1
#define LIS3DHTR_CTRL_REG3_CLICK_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_CLICK_ENABLE (0x80)  // enable

#define LIS3DHTR_CTRL_REG3_IA1_MASK (0x40)    //  IA1 interrupt on INT1
#define LIS3DHTR_CTRL_REG3_IA1_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_IA1_ENABLE (0x40)  // enable

#define LIS3DHTR_CTRL_REG3_IA2_MASK (0x20)    //  IA2 interrupt on INT1
#define LIS3DHTR_CTRL_REG3_IA2_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_IA2_ENABLE (0x20)  // enable

#define LIS3DHTR_CTRL_REG3_ZXYDA_MASK (0x10)    // ZYXDA interrupt on INT1
#define LIS3DHTR_CTRL_REG3_ZXYDA_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_ZXYDA_ENABLE (0x10)  // enable

#define LIS3DHTR_CTRL_REG3_321DA_MASK (0x08)    //  321DA interrupt on INT1
#define LIS3DHTR_CTRL_REG3_321DA_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_321DA_ENABLE (0x08)  // enable

#define LIS3DHTR_CTRL_REG3_WTM_MASK (0x04)    //  FIFO watermark interrupt on INT1
#define LIS3DHTR_CTRL_REG3_WTM_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_WTM_ENABLE (0x04)  // enable

#define LIS3DHTR_CTRL_REG3_OVERRUN_MASK (0x02)    //  FIFO overrun interrupt on INT1
#define LIS3DHTR_CTRL_REG3_OVERRUN_DISABLE (0x00) // disable
#define LIS3DHTR_CTRL_REG3_OVERRUN_ENABLE (0x02)  // enable

/**************************************************************************
    ACCELEROMETER CONTROL REGISTER 4 DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BDU_MASK (0x80)       // Block Data Update
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BDU_CONTINUOUS (0x00) // Continuous Update
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BDU_NOTUPDATED (0x80) // Output Registers Not Updated until MSB and LSB Read

#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BLE_MASK (0x40) // Big/Little Endian Data Selection
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BLE_LSB (0x00)  // Data LSB @ lower address
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_BLE_MSB (0x40)  // Data MSB @ lower address

#define LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_MASK (0x30) // Full-Scale Selection
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_2G (0x00)   // +/- 2G
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_4G (0x10)   // +/- 4G
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_8G (0x20)   // +/- 8G
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_16G (0x30)  // +/- 16G

#define LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_MASK (0x08)    // High Resolution Output Mode
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_DISABLE (0x00) // High Resolution Disable
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE (0x08)  // High Resolution Enable

#define LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_MASK (0x06)   // Self-Test Enable
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_NORMAL (0x00) // Normal Mode
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_0 (0x02)      // Self-Test 0
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_1 (0x04)      // Self-Test 1

#define LIS3DHTR_REG_ACCEL_CTRL_REG4_SIM_MASK (0x01)  // SPI Serial Interface Mode Selection
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_SIM_4WIRE (0x00) // 4-Wire Interface
#define LIS3DHTR_REG_ACCEL_CTRL_REG4_SIM_3WIRE (0x01) // 3-Wire Interface

/**************************************************************************
    ACCELEROMETER CONTROL REGISTER 5 DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_REBOOT_MASK (0x80)    // Reboot / reset the device.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_REBOOT_DISABLE (0x00) // Don't reboot the device.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_REBOOT_ENABLE (0x80)  // Reboot / reset the device.

#define LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_MASK (0x40)    // FIFO queue enable.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_DISABLE (0x00) // Disable the FIFO queue (default)
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_ENABLE (0x40)  // Enable the FIFO queue.

#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT1_MASK (0x08)    // Latch interrupts on pin 1.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT1_DISABLE (0x00) // No latch
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT1_ENABLE (0x08)  // Latch until the INT1_SRC register is read.

#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT1_MASK (0x04)    // 4D enable (depends on 6D bit on INT1_CFG).
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT1_DISABLE (0x00) // Disable 4D interrupts on this pin.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT1_ENABLE (0x04)  // Enable 4D interrupts on this pin.

#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT2_MASK (0x02)    // Latch interrupts on pin 2.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT2_DISABLE (0x00) // No latch
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_LIR_INT2_ENABLE (0x02)  // Latch until the INT2_SRC register is read.

#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT2_MASK (0x01)    // 4D enable (depends on 6D bit on INT2_CFG).
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT2_DISABLE (0x00) // Disable 4D interrupts on this pin.
#define LIS3DHTR_REG_ACCEL_CTRL_REG5_D4D_INT2_ENABLE (0x01)  // Enable 4D interrupts on this pin.

/**************************************************************************
    ACCELEROMETER STATUS 2 REGISTER DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_STATUS2_UPDATE_MASK (0x08) // Has New Data Flag Mask

/**************************************************************************
    ACCELEROMETER FIFO CONTROL REGISTER DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_MASK (0xC0)        // FIFO mode selection.
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_BYPASS (0x00)      // Bypass the FIFO (normal, default).
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_FIFO (0x40)        // FIFO mode.
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_STREAM (0x80)      // Stream mode.
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_STREAM_FIFO (0xC0) // Stream to FIFO mode.

#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_TRIGGER (0x20)      // Trigger selection.
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_TRIGGER_INT1 (0x00) // Trigger on int 1.
#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_TRIGGER_INT2 (0x20) // Trigger on int 2.

#define LIS3DHTR_REG_ACCEL_FIFO_CTRL_WATERMARK_MASK (0x1F) // High watermark mask.

/**************************************************************************
    ACCELEROMETER FIFO INTERRUPT SOURCE REGISTER DESCRIPTION
**************************************************************************/
#define LIS3DHTR_REG_ACCEL_FIFO_SRC_WTM_MASK (0x80)            // Bit to indicate watermark level exceeded.
#define LIS3DHTR_REG_ACCEL_FIFO_SRC_OVERRUN_MASK (0x40)        // Bit set when buffer is full.
#define LIS3DHTR_REG_ACCEL_FIFO_SRC_EMPTY_MASK (0x20)          // Bit set when all samples have been read.
#define LIS3DHTR_REG_ACCEL_FIFO_SRC_UNREAD_SAMPLES_MASK (0x1F) // Number of unread samples

enum power_type_t // power mode
{
    POWER_MODE_NORMAL = LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL,
    POWER_MODE_LOW = LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_LOW
};

enum scale_type_t // measurement rage
{
    LIS3DHTR_RANGE_2G = LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_2G,   //
    LIS3DHTR_RANGE_4G = LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_4G,   //
    LIS3DHTR_RANGE_8G = LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_8G,   //
    LIS3DHTR_RANGE_16G = LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_16G, //
};

enum odr_type_t // output data rate
{
    LIS3DHTR_DATARATE_POWERDOWN = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_PD,
    LIS3DHTR_DATARATE_1HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1,
    LIS3DHTR_DATARATE_10HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_10,
    LIS3DHTR_DATARATE_25HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_25,
    LIS3DHTR_DATARATE_50HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_50,
    LIS3DHTR_DATARATE_100HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_100,
    LIS3DHTR_DATARATE_200HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_200,
    LIS3DHTR_DATARATE_400HZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_400,
    LIS3DHTR_DATARATE_1_6KH = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_1_6K,
    LIS3DHTR_DATARATE_5KHZ = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_5K
};

/**
 * @brief  Struct to put raw data in.
 *
 * The packed attribute makes sure extra padding isn't added so we can cast directly from a byte
 * array. This may have issues on some systems?
 *
 */
struct values_type_t
{
    int16_t x;
    int16_t y;
    int16_t z;
};

const int gsdggf = sizeof(values_type_t);

template <class T>
class LIS3DHTR
{
public:
    LIS3DHTR();

    /**
     * @brief Checks if the accelerometer is connected and communicating correctly.
     */
    bool isConnection(void);

    /**
     * @brief Initialises the accelerometer with an SPI connection.
     *
     * @param comm the SPI bus to use.
     * @param sspin the chip select pin to use.
     * @param freq the frequency to use (10MHz) is the rated speed in the datasheet.
     */
    void begin(SPIClass &comm = SPI, uint8_t sspin = SS, uint32_t freq=10000000L);

    /**
     * @brief Initialises the accelerometer with an I2C connection.
     *
     * @param comm the I2C bus to use (some microcontrollers have multiple busses).
     * @param address the address of the accelerometer on the bus (either
     *                `LIS3DHTR_DEFAULT_ADDRESS` (0x18) or `LIS3DHTR_ADDRESS_UPDATED` (0x19)).
     */
    void begin(TwoWire &comm = Wire, uint8_t address = LIS3DHTR_DEFAULT_ADDRESS);

    /**
     * @brief Initialises the accelerometer with the default I2C bus connection.
     *
     * @param address the address of the accelerometer on the bus (either
     *                `LIS3DHTR_DEFAULT_ADDRESS` (0x18) or `LIS3DHTR_ADDRESS_UPDATED` (0x19)).
     */
    void begin(uint8_t address) { begin(Wire, address); };

    /**
     * @brief Sets the power mode of the accelerometer. This changes the number of bits used for each channel. L
     *
     * @param mode
     */
    void setPowerMode(power_type_t mode);

    /**
     * @brief Sets the full scale range of the accelerometer.
     */
    void setFullScaleRange(scale_type_t range);

    /**
     * @brief Sets the output data rate.
     *
     * @param odr the output data rate to request.
     */
    void setOutputDataRate(odr_type_t odr);

    /**
     * @brief Enable or disable high resolution output mode.
     *
     * @param enable whether to enable the mode.
     */
    void setHighSolution(bool enable);

    /**
     * @brief Checks if new readings are available.
     */
    bool available();

    /**
     * @brief Reads the X, Y and Z values from the accelerometer and converts them to Gs.
     *
     * @param x the X acceleration in G.
     * @param y the Y acceleration in G.
     * @param z the Z acceleration in G.
     */
    void getAcceleration(float *x, float *y, float *z);

    /**
     * @brief Reads the raw X, Y and Z values from the accelerometer.
     *
     * @param raw struct to place the X, Y and Z values in.
     */
    void getAccelerationRaw(values_type_t &raw);

    /**
     * @brief Reads only the X axis acceleration.
     *
     * @return float the acceleration in Gs.
     */
    float getAccelerationX(void);

    /**
     * @brief Reads only the Y axis acceleration.
     *
     * @return float the acceleration in Gs.
     */
    float getAccelerationY(void);

    /**
     * @brief Reads only the Z axis acceleration.
     *
     * @return float the acceleration in Gs.
     */
    float getAccelerationZ(void);

    void click(uint8_t c, uint8_t click_thresh, uint8_t limit = 10, uint8_t latency = 20, uint8_t window = 255);

    /**
     * @brief Reads the interrupt status register in the accelerometer.
     *
     * See the INT1_SRC (31h) in the datasheet. As a summary:
     *
     * - Bit 6: Interrupt active.
     * - Bit 5: Z high.
     * - Bit 4: Z low.
     * - Bit 3: Y high.
     * - Bit 2: Y low.
     * - Bit 1: X high.
     * - Bit 0: X low.
     *
     * @param flag pointer to a byte to place the value in.
     */
    void getIntStatus(uint8_t *flag);

    void setInterrupt(void);

    /**
     * @brief Enables the data ready (DRDY) interrupt.
     *
     * Interrupt 1 will go high when fresh data is available and low when it is read.
     *
     */
    void setDRDYInterrupt(void);

    /**
     * @brief Enables the temperature sensor.
     *
     */
    void openTemp();

    /**
     * @brief Disables the temperature sensor, but leaves the
     * ADCs operational.
     *
     */
    void closeTemp();

    /**
     * @brief Reads the channel 1 auxilliary ADC.
     */
    uint16_t readbitADC1(void);

    /**
     * @brief Reads the channel 2 auxilliary ADC.
     */
    uint16_t readbitADC2(void);

    /**
     * @brief Reads the channel 3 auxilliary ADC.
     *
     * If the temperature sensor is enabled, this could be reading the temperatue instead of
     * whatever is connected to the channel 3 pin. In this case, use `getTemperature()` to obtain
     * a nicely scaled output.
     */
    uint16_t readbitADC3(void);

    /**
     * @brief Obtains the temperature.
     *
     * Call `openTemp()` to enable temperature readign before calling this method.
     *
     * @return int16_t temperature estimate in degrees C.
     */
    int16_t getTemperature(void);

    /**
     * @brief Reads and returns the device ID. This should always be 0x33.
     */
    uint8_t getDeviceID(void);

    /**
     * @brief Resets the registers back to their defaults on first boot, followed by repeating the setup commands sent
     * by this library.
     *
     */
    void reset(void);

    /**
     * @brief Instructs the accelerometer to use its internal FIFO queue in stream mode.
     *
     * In stream mode, new values are placed on the end of the 32 level queue. When a given number
     * of values have been accrued (the high watermark + 1), an interrupt on pin 1 will be created.
     * This signals to the microcontroller that it can access data as a batch.
     *
     * One application of stream mode is high frequency sampling and removing the need for an
     * interrupt every single sample.
     *
     * See section "5.1.3 Stream mode" in the manual for more details.
     *
     * @param highWatermark when there are more samples in the FIFO buffer than this, an interrupt
     *                      will occur. When the number of samples drops to less than or equal this
     *                      value due to data being read, the interrupt will be cleared.
     */
    void setupFIFOStream(const uint8_t highWatermark);

    /**
     * @brief Reads raw data from the FIFO buffer.
     *
     * @param values array of structs to place the records in.
     * @param valuesToRead the number of records (X, Y and Z tuples) to read. Make sure this is the
     *                     same size or smaller than the length of values.
     */
    void readFIFORaw(values_type_t *values, uint8_t valuesToRead);

    /**
     * @brief Obtains the number of unread samples in the FIFO buffer.
     */
    uint8_t unreadFIFOSamples(void);

    operator bool();

private:
    /**
     * @brief Initialises the registers to something useful.
     *
     */
    void initRegisters(void);

    /**
     * @brief Reads a region of memory from the accelerometer.
     *
     * @param outputPointer pointer to a byte array to place the data in.
     * @param reg the address of the first register to read.
     * @param length the number of bytes to read. Make sure this is smaller than the length of the
     *               byte array at `outputPointer`; a buffer overflow will occur otherwise.
     */
    void readRegisterRegion(uint8_t *outputPointer, uint8_t reg, uint8_t length);

    /**
     * @brief Writes to a register on the accelerometer using the configurred connection method.
     *
     * @param reg the address of the register.
     * @param val the value to write.
     */
    void writeRegister(uint8_t reg, uint8_t val);

    /**
     * @brief Reads from a register on the accelerometer using the configurred connection method.
     *
     * @param reg the address of the register.
     * @return uint8_t the contents of the register.
     */
    uint8_t readRegister(uint8_t reg);

    /**
     * @brief Reads two registers from the accelerometer and combines these into a 16 bit integer.
     *
     * @param reg the address of the register.
     * @return uint16_t ther contents of the register.
     */
    uint16_t readRegisterInt16(uint8_t reg);

    /**
     * @brief Reads the registers for a given ADC and processes it into the expected number format.
     *
     * @param startReg the low register of the ADC channel.
     * @return uint16_t the ADC value.
     */
    uint16_t readbitADC(const uint8_t startReg);

    uint8_t devAddr;
    int16_t accRange;
    uint8_t commInterface;
    uint8_t chipSelectPin;
    SPIClass *_spi_com;
    SPISettings _settings;
    TwoWire *_wire_com;
};

#endif /*SEEED_LIS3DHTR_H*/
