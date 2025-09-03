#include <Wire.h>
#include <MPU6050.h>        // Electronic Cats library
#include <arduinoFFT.h>


MPU6050 mpu;


ArduinoFFT<double> FFT = ArduinoFFT<double>();


const int samples = 64;        // Must be a power of two for FFT
double vReal[samples];         // Real part of input
double vImag[samples];         // Imaginary part (init to 0)


void setup() {
  Serial.begin(115200);
  Wire.begin();


  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connected. Starting FFT...");
}


void loop() {
  int16_t ax, ay, az;
  int16_t gx_raw, gy_raw, gz_raw;


  // Sample sensor data at a fixed rate
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);


    // Convert raw gyro Z-axis to degrees per second (sensitivity 131 LSB/(°/s))
    double gz = (double)gz_raw / 131.0;


    vReal[i] = gz;
    vImag[i] = 0;  // Imaginary part is zero for real input


    delay(20);  // Sampling frequency ~50Hz
  }


  // Perform FFT
  FFT.windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, samples);


  // Find dominant frequency bin index (excluding DC component at index 0)
  int peakIndex = 1;
  double peakValue = vReal[1];
  for (int i = 2; i < samples / 2; i++) {
    if (vReal[i] > peakValue) {
      peakValue = vReal[i];
      peakIndex = i;
    }
  }


  // Calculate frequency in Hz
  double samplingFreq = 50.0;  // 50 Hz sampling frequency
  double freq = (peakIndex * samplingFreq) / samples;


  Serial.print("Dominant frequency: ");
  Serial.print(freq, 2);
  Serial.println(" Hz");


  delay(500);  // Print every 0.5s
}
