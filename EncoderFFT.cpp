
#include <FlexCAN_T4.h>
#include <arduinoFFT.h>

#define NUM_SENSORS 8   // Adjust if needed
#define SAMPLES 128     // FFT sample size (power of 2)
#define SAMPLING_FREQUENCY 10000

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

arduinoFFT FFT = arduinoFFT();
double real[SAMPLES];
double imag[SAMPLES];

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
    Serial.begin(115200);
    can1.begin();
    can1.setBaudRate(500000);
}

void loop() {
    for (int i = 0; i < SAMPLES; i++) {
        double totalVoltage = 0;
        for (int j = 0; j < NUM_SENSORS; j++) {
            totalVoltage += analogRead(sensorPins[j]);
        }
        real[i] = totalVoltage / NUM_SENSORS;
        imag[i] = 0;
        delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
    }

    FFT.Windowing(real, SAMPLES, FFT_WIN_HAMMING, FFT_FORWARD);
    FFT.Compute(real, imag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(real, imag, SAMPLES);

    int peakIndex = FFT.MajorPeak(real, SAMPLES, SAMPLING_FREQUENCY);
    double frequency = (double)peakIndex * SAMPLING_FREQUENCY / SAMPLES;
    double phaseShift = atan2(imag[peakIndex], real[peakIndex]);

    double position = (phaseShift * 180 / PI) + 180;

    // Send position data via CAN
    msg.id = 0x100;  
    msg.len = 2;
    msg.buf[0] = (uint8_t)(position) & 0xFF;  // Low byte
    msg.buf[1] = (uint8_t)(position >> 8) & 0xFF;  // High byte
    can1.write(msg);

    Serial.print("Position: ");
    Serial.print(position);
    Serial.println(" degrees");

    delay(10); // Adjust delay for desired update rate
}
