/*
 * A library for Grove - 3-Axis Digital Accelerometer ±2g to 16g Ultra-low Power(LIS3DHTR)
 *
 * Copyright (c) 2019 seeed technology co., ltd.
 * Author      : Hongtai Liu (lht856@foxmail.com)
 * Create Time : July 2019
 * Change Log  :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software istm
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS INcommInterface
 * THE SOFTWARE.
 */

#include "LIS3DHTR.h"

#ifdef ARDUINO_ARCH_ESP32
// Need to provide and actual type definition for spi_t when using DMA.
// https://github.com/espressif/arduino-esp32/issues/1427#issuecomment-391364214
#include "soc/spi_struct.h"
struct spi_struct_t
{
    spi_dev_t *dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    SemaphoreHandle_t lock;
#endif
    uint8_t num;
    int8_t sck;
    int8_t miso;
    int8_t mosi;
    int8_t ss;
};
#endif

template <class T>
LIS3DHTR<T>::LIS3DHTR()
{
}

template <class T>
void LIS3DHTR<T>::begin(SPIClass &comm, uint8_t sspin, uint32_t freq)
{
    chipSelectPin = sspin;
    _spi_com = &comm;
    _wire_com = NULL;
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);

    // Maximum SPI frequency is 10MHz, Data is read and written MSb first,
    // Data is captured on rising edge of clock (CPHA = 0)
    // Base value of the clock is HIGH (CPOL = 1)
    // MODE3 for 328p operation
    _settings._clock = freq;
    _settings._bitOrder = MSBFIRST;
    _settings._dataMode = SPI_MODE3;

    // start the SPI library:
    _spi_com->begin();

    delay(200);

    initRegisters();
}

template <class T>
void LIS3DHTR<T>::begin(TwoWire &wire, uint8_t address)
{
    _wire_com = &wire;
    _wire_com->begin();
    _spi_com = NULL;
    devAddr = address;

    initRegisters();
}

template <class T>
bool LIS3DHTR<T>::available()
{
    uint8_t status = 0;
    status = readRegister(LIS3DHTR_REG_ACCEL_STATUS2);
    status &= LIS3DHTR_REG_ACCEL_STATUS2_UPDATE_MASK;
    return status;
}

template <class T>
void LIS3DHTR<T>::openTemp()
{
    uint8_t config5 = LIS3DHTR_REG_TEMP_ADC_PD_ENABLED |
                      LIS3DHTR_REG_TEMP_TEMP_EN_ENABLED;

    writeRegister(LIS3DHTR_REG_TEMP_CFG, config5);
    delay(LIS3DHTR_CONVERSIONDELAY);
}

template <class T>
void LIS3DHTR<T>::closeTemp()
{
    uint8_t config5 = LIS3DHTR_REG_TEMP_ADC_PD_ENABLED |
                      LIS3DHTR_REG_TEMP_TEMP_EN_DISABLED;

    writeRegister(LIS3DHTR_REG_TEMP_CFG, config5);
    delay(LIS3DHTR_CONVERSIONDELAY);
}

template <class T>
int16_t LIS3DHTR<T>::getTemperature(void)
{
    // Datasheet says 1 digit per degree C in 8 bit resolution.
    int16_t result = ((int16_t)readRegisterInt16(LIS3DHTR_REG_OUT_ADC3_L)) / 256;
    result += 25;
    return result;
}

template <class T>
bool LIS3DHTR<T>::isConnection(void)
{
    return (getDeviceID() == 0x33);
}

template <class T>
uint8_t LIS3DHTR<T>::getDeviceID(void)
{
    return readRegister(LIS3DHTR_REG_ACCEL_WHO_AM_I);
}

template <class T>
void LIS3DHTR<T>::setPowerMode(power_type_t mode)
{
    uint8_t data = 0;

    data = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1);

    data &= ~LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_MASK;
    data |= mode;

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1, data);
    delay(LIS3DHTR_CONVERSIONDELAY);
}

template <class T>
void LIS3DHTR<T>::setFullScaleRange(scale_type_t range)
{
    uint8_t data = 0;

    data = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG4);

    data &= ~LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_MASK;
    data |= range;

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG4, data);
    delay(LIS3DHTR_CONVERSIONDELAY);

    switch (range)
    {
    case LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_16G:
        accRange = 1280;
        break;
    case LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_8G:
        accRange = 3968;
        break;
    case LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_4G:
        accRange = 7282;
        break;
    case LIS3DHTR_REG_ACCEL_CTRL_REG4_FS_2G:
        accRange = 16000;
        break;
    default:
        break;
    }
}

template <class T>
void LIS3DHTR<T>::setOutputDataRate(odr_type_t odr)
{
    uint8_t data = 0;

    data = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1);

    data &= ~LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_MASK;
    data |= odr;

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1, data);
    delay(LIS3DHTR_CONVERSIONDELAY);
}

template <class T>
void LIS3DHTR<T>::getAcceleration(float *x, float *y, float *z)
{
    values_type_t raw;
    getAccelerationRaw(raw);

    // Conversion of the result
    // 16-bit signed result for X-Axis Acceleration Data of LIS3DHTR
    *x = (float)raw.x / accRange;
    // 16-bit signed result for Y-Axis Acceleration Data of LIS3DHTR
    *y = (float)raw.y / accRange;
    // 16-bit signed result for Z-Axis Acceleration Data of LIS3DHTR
    *z = (float)raw.z / accRange;
}

template <class T>
void LIS3DHTR<T>::getAccelerationRaw(values_type_t &raw)
{
    // Read the Data
    readRegisterRegion((uint8_t *)&raw, LIS3DHTR_REG_ACCEL_OUT_X_L, 6);
}

template <class T>
float LIS3DHTR<T>::getAccelerationX(void)
{
    // Read the Accelerometer
    uint8_t xAccelLo, xAccelHi;
    int16_t x;

    // Read the Data
    // Reading the Low X-Axis Acceleration Data Register
    xAccelLo = readRegister(LIS3DHTR_REG_ACCEL_OUT_X_L);
    // Reading the High X-Axis Acceleration Data Register
    xAccelHi = readRegister(LIS3DHTR_REG_ACCEL_OUT_X_H);
    // Conversion of the result
    // 16-bit signed result for X-Axis Acceleration Data of LIS3DHTR
    x = (int16_t)((xAccelHi << 8) | xAccelLo);

    return (float)x / accRange;
}

template <class T>
float LIS3DHTR<T>::getAccelerationY(void)
{
    // Read the Accelerometer
    uint8_t yAccelLo, yAccelHi;
    int16_t y;

    // Reading the Low Y-Axis Acceleration Data Register
    yAccelLo = readRegister(LIS3DHTR_REG_ACCEL_OUT_Y_L);
    // Reading the High Y-Axis Acceleration Data Register
    yAccelHi = readRegister(LIS3DHTR_REG_ACCEL_OUT_Y_H);
    // Conversion of the result
    // 16-bit signed result for Y-Axis Acceleration Data of LIS3DHTR
    y = (int16_t)((yAccelHi << 8) | yAccelLo);

    return (float)y / accRange;
}

template <class T>
float LIS3DHTR<T>::getAccelerationZ(void)
{
    // Read the Accelerometer
    uint8_t zAccelLo, zAccelHi;
    int16_t z;

    // Reading the Low Z-Axis Acceleration Data Register
    zAccelLo = readRegister(LIS3DHTR_REG_ACCEL_OUT_Z_L);
    // Reading the High Z-Axis Acceleration Data Register
    zAccelHi = readRegister(LIS3DHTR_REG_ACCEL_OUT_Z_H);
    // Conversion of the result
    // 16-bit signed result for Z-Axis Acceleration Data of LIS3DHTR
    z = (int16_t)((zAccelHi << 8) | zAccelLo);

    return (float)z / accRange;
}

template <class T>
void LIS3DHTR<T>::setHighSolution(bool enable)
{
    uint8_t data = 0;
    data = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG4);

    data = enable ? data | LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE : data & ~LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_ENABLE;

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG4, data);
    return;
}

template <class T>
uint16_t LIS3DHTR<T>::readbitADC1(void)
{
    return readbitADC(LIS3DHTR_REG_OUT_ADC1_L);
}
template <class T>
uint16_t LIS3DHTR<T>::readbitADC2(void)
{
    return readbitADC(LIS3DHTR_REG_OUT_ADC2_L);
}

template <class T>
uint16_t LIS3DHTR<T>::readbitADC3(void)
{
    return readbitADC(LIS3DHTR_REG_OUT_ADC3_L);
}

template <class T>
void LIS3DHTR<T>::writeRegister(uint8_t reg, uint8_t val)
{
    if (_spi_com != NULL)
    {
        _spi_com->beginTransaction(_settings);
        digitalWrite(chipSelectPin, LOW);
        _spi_com->transfer(reg);
        _spi_com->transfer(val);
        digitalWrite(chipSelectPin, HIGH);
        _spi_com->endTransaction();
    }
    else
    {
        _wire_com->beginTransmission(devAddr);
        _wire_com->write(reg);
        _wire_com->write(val);
        _wire_com->endTransmission();
    }
}

template <class T>
void LIS3DHTR<T>::initRegisters(void)
{
    uint8_t config5 = LIS3DHTR_REG_TEMP_ADC_PD_ENABLED |
                      LIS3DHTR_REG_TEMP_TEMP_EN_DISABLED;

    writeRegister(LIS3DHTR_REG_TEMP_CFG, config5);
    delay(LIS3DHTR_CONVERSIONDELAY);

    uint8_t config1 = LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL | // Normal Mode
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE | // Acceleration Z-Axis Enabled
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE | // Acceleration Y-Axis Enabled
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE;

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1, config1);
    delay(LIS3DHTR_CONVERSIONDELAY);

    uint8_t config4 = LIS3DHTR_REG_ACCEL_CTRL_REG4_BDU_NOTUPDATED | // Continuous Update
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_BLE_LSB |        // Data LSB @ lower address
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_HS_DISABLE |     // High Resolution Disable
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_ST_NORMAL |      // Normal Mode
                      LIS3DHTR_REG_ACCEL_CTRL_REG4_SIM_4WIRE;       // 4-Wire Interface

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG4, config4);

    // Disable FIFO
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, 0x00);

    delay(LIS3DHTR_CONVERSIONDELAY);

    setFullScaleRange(LIS3DHTR_RANGE_16G);
    setOutputDataRate(LIS3DHTR_DATARATE_400HZ);
}

template <class T>
void LIS3DHTR<T>::readRegisterRegion(uint8_t *outputPointer, uint8_t reg, uint8_t length)
{

    // define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;

    if (_spi_com != NULL)
    {
        _spi_com->beginTransaction(_settings);
        digitalWrite(chipSelectPin, LOW);
        _spi_com->transfer(reg | 0x80 | 0x40); // Ored with "read request" bit and "auto increment" bit
        while (i < length)                     // slave may send less than requested
        {
            c = _spi_com->transfer(0x00); // receive a byte as character
            *outputPointer = c;
            outputPointer++;
            i++;
        }
        digitalWrite(chipSelectPin, HIGH);
        _spi_com->endTransaction();
    }
    else
    {

        _wire_com->beginTransmission(devAddr);
        reg |= 0x80; // turn auto-increment bit on, bit 7 for I2C
        _wire_com->write(reg);
        _wire_com->endTransmission(false);
        _wire_com->requestFrom(devAddr, length);

        while ((_wire_com->available()) && (i < length)) // slave may send less than requested
        {
            c = _wire_com->read(); // receive a byte as character
            *outputPointer = c;
            outputPointer++;
            i++;
        }
    }
}

template <class T>
uint16_t LIS3DHTR<T>::readRegisterInt16(uint8_t reg)
{

    uint8_t myBuffer[2];
    readRegisterRegion(myBuffer, reg, 2);
    uint16_t output = myBuffer[0] | uint16_t(myBuffer[1] << 8);

    return output;
}

template <class T>
uint16_t LIS3DHTR<T>::readbitADC(const uint8_t startReg)
{
    int16_t signedADC = (int16_t)readRegisterInt16(startReg);
    signedADC = 0 - signedADC;
    uint16_t unsignedADC = signedADC + 32768;
    return unsignedADC >> 6;
}

template <class T>
uint8_t LIS3DHTR<T>::readRegister(uint8_t reg)
{
    uint8_t data;

    readRegisterRegion(&data, reg, 1);

    return data;
}

template <class T>
void LIS3DHTR<T>::click(uint8_t c, uint8_t click_thresh, uint8_t limit, uint8_t latency, uint8_t window)
{
    if (!c)
    {
        uint8_t r = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG3);
        r &= ~(0x80); // turn off I1_CLICK
        writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG3, r);
        writeRegister(LIS3DHTR_REG_ACCEL_CLICK_CFG, 0);
        return;
    }
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG3, 0x80);
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, 0x08);

    if (c == 1)
    {
        writeRegister(LIS3DHTR_REG_ACCEL_CLICK_CFG, 0x15);
    }
    if (c == 2)
    {
        writeRegister(LIS3DHTR_REG_ACCEL_CLICK_CFG, 0x2A);
    }

    writeRegister(LIS3DHTR_REG_ACCEL_CLICK_THS, click_thresh);
    writeRegister(LIS3DHTR_REG_ACCEL_TIME_LIMIT, limit);
    writeRegister(LIS3DHTR_REG_ACCEL_TIME_LATENCY, latency);
    writeRegister(LIS3DHTR_REG_ACCEL_TIME_WINDOW, window);
}

template <class T>
void LIS3DHTR<T>::getIntStatus(uint8_t *flag)
{
    *flag = readRegister(LIS3DHTR_REG_ACCEL_INT1_SRC);
}

template <class T>
void LIS3DHTR<T>::setInterrupt(void)
{
    uint8_t data = 0;
    uint8_t config1 = LIS3DHTR_REG_ACCEL_CTRL_REG1_AODR_50 |
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_LPEN_NORMAL | // Normal Mode
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AZEN_ENABLE | // Acceleration Z-Axis Enabled
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AYEN_ENABLE | // Acceleration Y-Axis Enabled
                      LIS3DHTR_REG_ACCEL_CTRL_REG1_AXEN_ENABLE;
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG1, config1); // (50 Hz),  X/Y/Z-axis enable

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG2, 0x00); //

    uint8_t config3 = LIS3DHTR_CTRL_REG3_IA1_ENABLE;
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG3, config3); // IA1 interrupt

    setFullScaleRange(LIS3DHTR_RANGE_8G);

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, 0x00); // Latch interrupt request

    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG6, 0x42); // IA1, active-low  Enable interrupt 1 function on INT2 pin

    writeRegister(LIS3DHTR_REG_ACCEL_INT1_THS, 0x50); // set Threshold，2g =>16mg/LSB，4g => 32mg/LSB，8g => 62mg/LSB，16g => 186mg/LSB

    writeRegister(LIS3DHTR_REG_ACCEL_INT1_DURATION, 0);

    data = readRegister(LIS3DHTR_REG_ACCEL_INT1_SRC); // clear interrupt flag
    (void)data;                                       // UNUSED

    writeRegister(LIS3DHTR_REG_ACCEL_INT1_CFG, 0x2a); // trigger when ZHIE/YHIE/XHIE
}

template <class T>
void LIS3DHTR<T>::setDRDYInterrupt(void)
{
    // Write only as we wish to disable all other interrupts.
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG3, LIS3DHTR_CTRL_REG3_ZXYDA_ENABLE); // Enable ZYXDA
}

template <class T>
void LIS3DHTR<T>::reset(void)
{
    // No point reading current register values as we are about to reset it to defaults anyway.
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, LIS3DHTR_REG_ACCEL_CTRL_REG5_REBOOT_ENABLE);
    delay(LIS3DHTR_CONVERSIONDELAY);
    // Go back to a known useful state.
    initRegisters();
}

template <class T>
void LIS3DHTR<T>::setupFIFOStreamHWM(const uint8_t highWatermark)
{
    // Enable the FIFO.
    uint8_t reg5 = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5);
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, reg5 | LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_ENABLE);

    // Setup stream mode with the given high watermark.
    writeRegister(
        LIS3DHTR_REG_ACCEL_FIFO_CTRL,
        LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_STREAM | LIS3DHTR_REG_ACCEL_FIFO_CTRL_TRIGGER_INT1 |
            (highWatermark & LIS3DHTR_REG_ACCEL_FIFO_CTRL_WATERMARK_MASK));

    // Set the interrupt to occur only on high watermark.
    writeRegister(
        LIS3DHTR_REG_ACCEL_CTRL_REG3,
        LIS3DHTR_CTRL_REG3_CLICK_DISABLE | LIS3DHTR_CTRL_REG3_IA1_DISABLE |
            LIS3DHTR_CTRL_REG3_IA2_DISABLE | LIS3DHTR_CTRL_REG3_ZXYDA_DISABLE |
            LIS3DHTR_CTRL_REG3_321DA_DISABLE | LIS3DHTR_CTRL_REG3_WTM_ENABLE |
            LIS3DHTR_CTRL_REG3_OVERRUN_DISABLE);
}

template <class T>
void LIS3DHTR<T>::setupFIFOStreamOVRN()
{
    // Enable the FIFO.
    uint8_t reg5 = readRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5);
    writeRegister(LIS3DHTR_REG_ACCEL_CTRL_REG5, reg5 | LIS3DHTR_REG_ACCEL_CTRL_REG5_FIFO_ENABLE);

    // Setup stream mode
    writeRegister(
        LIS3DHTR_REG_ACCEL_FIFO_CTRL,
        LIS3DHTR_REG_ACCEL_FIFO_CTRL_MODE_STREAM | LIS3DHTR_REG_ACCEL_FIFO_CTRL_TRIGGER_INT1);

    // Set the interrupt to occur only on overrun.
    writeRegister(
        LIS3DHTR_REG_ACCEL_CTRL_REG3,
        LIS3DHTR_CTRL_REG3_CLICK_DISABLE | LIS3DHTR_CTRL_REG3_IA1_DISABLE |
            LIS3DHTR_CTRL_REG3_IA2_DISABLE | LIS3DHTR_CTRL_REG3_ZXYDA_DISABLE |
            LIS3DHTR_CTRL_REG3_321DA_DISABLE | LIS3DHTR_CTRL_REG3_WTM_DISABLE |
            LIS3DHTR_CTRL_REG3_OVERRUN_ENABLE);
}

template <class T>
void LIS3DHTR<T>::readFIFORaw(values_type_t *values, uint8_t valuesToRead)
{
    readRegisterRegion((uint8_t *)values, LIS3DHTR_REG_ACCEL_OUT_X_L, 6 * valuesToRead);
}

template <class T>
uint8_t LIS3DHTR<T>::unreadFIFOSamples(void)
{
    return readRegister(LIS3DHTR_REG_ACCEL_FIFO_SRC) & LIS3DHTR_REG_ACCEL_FIFO_SRC_UNREAD_SAMPLES_MASK;
}

#ifdef ARDUINO_ARCH_ESP32
template <class T>
void LIS3DHTR<T>::startFIFORawDMA(const uint8_t valuesToRead)
{
    if (_spi_com != NULL && valuesToRead <= 10) // 64 byte per transfer hard limit set in the spi library (60 bytes data, 1 byte address).
    {
        // Only bother continueing if we are using SPI (there will be a way to implement this with I2C later).
        // _spi_com->beginTransaction(_settings); // Mutexes mess with ISRs - Hopefully will be set up from before?
        digitalWrite(chipSelectPin, LOW);
        spi_t *spi = _spi_com->bus();

        // Set the number of bits to send and receive.
        const uint32_t bits = ((6 * valuesToRead + 1) * 8) - 1;
        spi->dev->mosi_dlen.usr_mosi_dbitlen = bits; // Only wish to write 7+1 bits for the address.
        spi->dev->miso_dlen.usr_miso_dbitlen = bits; // Number of bits to read.

        // Set the register to write.
        const uint8_t writeAddress = LIS3DHTR_REG_ACCEL_OUT_X_L | 0x80 | 0x40; // Ored with "read request" bit and "auto increment" bit
#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
        spi->dev->data_buf[0].val = writeAddress;
#else
        spi->dev->data_buf[0] = writeAddress;
#endif

        // Start writing and reading.
#if CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
        spi->dev->cmd.update = 1;
        while (spi->dev->cmd.update)
            ;
#endif
        spi->dev->cmd.usr = 1;
    }
}

template <class T>
bool LIS3DHTR<T>::dmaFinished()
{
    return !(_spi_com->bus()->dev->cmd.usr);
}

template <class T>
void LIS3DHTR<T>::finishFIFORawDMA(values_type_t *values, uint8_t valuesToRead)
{
    // Wait until the SPI write is finished just in case it isn't.
    spi_t *spi = _spi_com->bus();
    while (spi->dev->cmd.usr)
        ;

    // Copy the data into a buffer we can easily access / cast later.
    if (valuesToRead > 16)
    {
        valuesToRead = 16;
    }
    uint32_t buf[16];
    for (uint32_t i = 0; i < valuesToRead; i++)
    {
#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
        buf[i] = spi->dev->data_buf[i].val; // copy spi fifo to buffer
#else
        buf[i] = spi->dev->data_buf[i]; // copy spi fifo to buffer
#endif
    }

    // Copy into the main buffer now we don't have issues with arrays of structs.
    memcpy(values, buf + 1, 6 * valuesToRead);

    // Tidy up
    digitalWrite(chipSelectPin, HIGH);
    // _spi_com->endTransaction();
}
#endif

template <class T>
LIS3DHTR<T>::operator bool() { return isConnection(); }

template class LIS3DHTR<SPIClass>;
template class LIS3DHTR<TwoWire>;
