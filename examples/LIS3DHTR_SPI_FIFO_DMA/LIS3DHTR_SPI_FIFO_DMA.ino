// This example shows high speed data acquisition using the FIFO buffer using
// direct memory access (DMA). This example is intended for use with ESP32
// series microcontrollers only as implementations for other systems have not
// been implemented yet. This allows data points to be retrieved in batches,
// reducing the overheads when using interrupts. The batch size is limited to
// 10 samples at a time at most due to the size of the internal DMA buffer.
// DMA allows the microcontroller to offload data from the accelerometer in the
// background without requiring CPU time.

#include "LIS3DHTR.h"

// Pin definitions. Change these to suit your microcontroller and wiring.
#define PIN_LIS_CS 5    // 10 in other examples
#define PIN_LIS_INT1 27 // Should be interrupt capable.

#ifndef LED_BUILTIN
#define LED_BUILTIN -1 // In case LED_BUILTIN isn't defined.
#endif

// The number of readings to obtain before the interrupt is generated. This can be a maximum of 10 due to the size of the DMA buffer on the microcontroller side.
#define SAMPLES_PER_BATCH 10

// If defined, serial printing is disabled and accelerometer settings are set for 5kHz data
// acquisition. Connect a frequency counter to the LED_BUILTIN pin. Multiply the reported frequency
// by SAMPLES_PER_BATCH to obtain the actual sample frequency.
// #define FAST_DEMO

LIS3DHTR<SPIClass> LIS; // SPI

// Comment out things that won't compile correctly on non-esp32 platforms.
#ifdef ARDUINO_ARCH_ESP32
/**
 * @brief Task handle to send notifications to on interrupt.
 *
 */
volatile TaskHandle_t accelTask;

/**
 * @brief Interrupt that gets called when the high watermark is reached.
 *
 */
void IRAM_ATTR lisISR()
{
    // Check that we aren't sending items from the last interrupt. Ignore if so.
    if (LIS.dmaFinished())
    {
        digitalWrite(LED_BUILTIN, HIGH);

        // Start the transaction now using the DMA hardware.
        LIS.startFIFORawDMA(SAMPLES_PER_BATCH); // TODO: Use an interrupt when this is done.

        // Send a notification to the accelerometer's task.
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(accelTask, micros(), eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
#endif

void setup()
{
    // Initialise serial for logging.
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("LIS3DHTR FIFO DMA demonstration");
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialise the accelerometer.
    LIS.begin(SPI, PIN_LIS_CS, 1000000);
    delay(100);

    if (!LIS)
    {
        Serial.println("LIS3DHTR didn't connect.");
        while (1)
            ;
    }

    // Set the full scale range and output data rate.
    LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_4G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_8G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_16G);

#ifndef FAST_DEMO
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_1HZ);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_10HZ);
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_25HZ);
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_100HZ);
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_200HZ);
    // LIS.setOutputDataRate(LIS3DHTR_DATARATE_1_6KHZ);

#else
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_5KHZ);

    // For 5kHz acquisition, low power mode is required.
    LIS.setPowerMode(POWER_MODE_LOW);
#endif

#ifdef ARDUINO_ARCH_ESP32
    // Save the current task handle into a global variable so we can send notifications to it.
    accelTask = xTaskGetCurrentTaskHandle();

    // Set the accelerometer to use stream mode. Stream mode overwrites the oldest data when the
    // queue becomes full, whereas the other FIFO modes have different behaviours.
    LIS.setupFIFOStreamHWM(SAMPLES_PER_BATCH - 1);
    LIS.clearFIFO();
    attachInterrupt(digitalPinToInterrupt(PIN_LIS_INT1), lisISR, RISING);
#else
    Serial.println("Reading the FIFO buffer using DMA has not been implemented yet for this platform!.\r\n\
    Please implement this functionality or look at the non-DMA examples.");
#endif
}

void loop()
{
#ifdef ARDUINO_ARCH_ESP32
    // Wait for a notification that the interrupt has been received.
    uint32_t time;
    if (xTaskNotifyWait(0x0, 0xffffffff, &time, 5000 / portTICK_PERIOD_MS))
    {
        // We received a notification. Now wait for the transaction to finish if it hasn't already.
        // Calling `LIS.finishFIFORawDMA()` will block until finished
        while (!LIS.dmaFinished())
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            yield();
        }
        digitalWrite(LED_BUILTIN, HIGH);

        // Copy the DMA buffer into more readily accessible memory.
        values_type_t values[SAMPLES_PER_BATCH];
        LIS.finishFIFORawDMA(values, SAMPLES_PER_BATCH);

#ifndef FAST_DEMO
        // Print the data in a vaguely JSON like format (there won't be nearly enough bandwidth on
        // the serial port at higher data rates for this to work).
        Serial.printf("%lu: {\r\n", time);
        for (uint8_t i = 0; i < (SAMPLES_PER_BATCH); i++)
        {
            // Serial.printf("  %d: [%5hd, %5hd, %5hd],\r\n", i, values[i].x, values[i].y, values[i].z);
            Serial.printf("  %d: [%04hx, %04hx, %04hx],\r\n", i, values[i].x, values[i].y, values[i].z);
        }
        Serial.println("}");

        // Check how many samples we have left to read afterwards (not using DMA, will block).
        Serial.printf("After reading there are %d unread samples.\r\n\r\n", LIS.unreadFIFOSamples());
#endif
    }
    else
    {
        // The notification timed out. This means we didn't receive an interrupt from the accelerometer in a timely manner.
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Did not receive an interrupt from the accelerometer. May try removing and re-applying power.");
    }
#endif
    // Turn off the LED to show we are ready for the next interrupt.
    digitalWrite(LED_BUILTIN, LOW);
}