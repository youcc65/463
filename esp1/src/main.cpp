#include <Arduino.h>

#define PULSE_PIN 3            // ADC pin connected to Pulse Sensor (GPIO 4)
#define SAMPLING_RATE_MS 10    // Sampling interval in milliseconds
#define THRESHOLD 2500          // Threshold value to detect a peak (adjust as needed)

unsigned long lastBeatTime = 0; // Time of the last beat detected
int bpm = 0;                    // Beats per minute
bool isPeakDetected = false;

void setup() {
    Serial.begin(115200);  // Start serial communication at 115200 baud rate
    pinMode(PULSE_PIN, INPUT);  // Set the pulse pin as input
}

void loop() {
    static int signal;
    static unsigned long currentTime;
    static unsigned long lastSampleTime = 0;

    // Sampling at defined intervals
    currentTime = millis();
    if (currentTime - lastSampleTime >= SAMPLING_RATE_MS) {
        lastSampleTime = currentTime;

        // Read the analog value from the pulse sensor
        signal = analogRead(PULSE_PIN);

        
        // Print the current sensor value
        //Serial.print("Sensor Value: ");
        //Serial.println(signal);

        // Detect a beat
        if (signal > THRESHOLD && !isPeakDetected) {
            isPeakDetected = true;  // Mark that a peak has been detected

            // Calculate the time between beats (in ms)
            unsigned long timeBetweenBeats = currentTime - lastBeatTime;
            lastBeatTime = currentTime;

            // Calculate BPM: 60000 ms in a minute
            bpm = 60000 / timeBetweenBeats;

            Serial.print("Heart rate: ");
            Serial.print(bpm);
            Serial.println(" BPM");
        }

        // If signal drops below the threshold, reset peak detection
        if (signal < THRESHOLD) {
            isPeakDetected = false;
        }
    }

    delay(30); // Short delay to prevent overwhelming the CPU
}



