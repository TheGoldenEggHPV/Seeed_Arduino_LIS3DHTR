// This example shows high speed data acquisition using the FIFO buffer.
// This allows data points to be retrieved in batches, reducing the overheads when using interrupts.

#include "LIS3DHTR.h"

// Pin definitions. Change these to suit your microcontroller and wiring.
#define PIN_LIS_CS 5    // 10 in other examples
#define PIN_LIS_INT1 33 // Should be interrupt capable.

#ifndef LED_BUILTIN
    #define LED_BUILTIN 2 // In case LED_BUILTIN isn't defined.
#endif

// The number of readings to obtain before the interrupt is generated. Ideally set this low enough
// that the buffer does not overflow between the interrupt occurring and data being read out.
#define SAMPLES_PER_BATCH 25

// If defined, serial printing is disabled and accelerometer settings are set for 5kHz data
// acquisition. Connect a frequency counter to the LED_BUILTIN pin. Multiply the reported frequency
// by SAMPLES_PER_BATCH to obtain the actual sample frequency.
// #define FAST_DEMO

volatile bool watermarkReached = false; // Flag to signify when there is sufficient data to read.
volatile uint32_t interruptTime;

LIS3DHTR<SPIClass> LIS; // SPI

// Hopefully allows this code to compile on non ESP32 platforms.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/**
 * @brief Interrupt that gets called when the high watermark is reached.
 *
 */
void IRAM_ATTR lisISR()
{
    // If using freeRTOS, consider notifications instead.
    watermarkReached = true;
    interruptTime = micros();
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    // Initialise serial for logging.
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("LIS3DHTR FIFO demonstration");
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialise the accelerometer.
    LIS.begin(SPI, PIN_LIS_CS);
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

    // Set the accelerometer to use stream mode. Stream mode overwrites the oldest data when the
    // queue becomes full, whereas the other FIFO modes have different behaviours.
    attachInterrupt(digitalPinToInterrupt(PIN_LIS_INT1), lisISR, RISING);
    LIS.setupFIFOStreamHWM(SAMPLES_PER_BATCH - 1);
}

void loop()
{
    if (watermarkReached)
    {
#ifndef FAST_DEMO
        // Check how many samples we have to read.
        Serial.print("Interrupt occurred with ");
        Serial.print(LIS.unreadFIFOSamples());
        Serial.println(" unread samples");
#endif

        // An interrupt has occurred and data is ready. Read the buffer.
        values_type_t values[SAMPLES_PER_BATCH];
        LIS.readFIFORaw(values, SAMPLES_PER_BATCH);

#ifndef FAST_DEMO
        // Print the data in a vaguely JSON like format (there won't be nearly enough bandwidth on
        // the serial port at higher data rates for this to work).
        Serial.print(interruptTime);
        Serial.println(": {");
        for (uint8_t i = 0; i < (SAMPLES_PER_BATCH); i++)
        {
            Serial.print("  ");
            Serial.print(i);
            Serial.print(": [");
            Serial.print(values[i].x);
            Serial.print(",");
            Serial.print(values[i].y);
            Serial.print(",");
            Serial.print(values[i].z);
            Serial.println("],");
        }
        Serial.println("}");

        // Check how many samples we have left to read afterwards.
        Serial.print("After reading there are ");
        Serial.print(LIS.unreadFIFOSamples());
        Serial.println(" unread samples.");
#endif

        // Turn off the LED to show we are ready for the next interrupt.
        digitalWrite(LED_BUILTIN, LOW);
        watermarkReached = false;
    }
    yield();
}