#include <Wire.h>
#include <MPU6050.h>
#include <arduinoFFT.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>


MPU6050 mpu;


// FFT settings
const int samples = 64; // power of two, determines frequency resolution (fs/samples)
ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vRealX[samples], vImagX[samples];
float vRealY[samples], vImagY[samples];
float vRealZ[samples], vImagZ[samples];


// RMS window
const int windowSize = 50;
float gxWindow[windowSize] = {0}, gyWindow[windowSize] = {0}, gzWindow[windowSize] = {0};
int rmsIndex = 0;
bool rmsFilled = false;


// Tremor band definitions
const float fs = 50.0f;        // target sampling frequency in Hz
const float f_low = 3.0f;      // high-pass cutoff (Hz)
const float f_high = 12.0f;    // low-pass cutoff (Hz)


// Per-axis Butterworth filters: high-pass then low-pass for bandpass
auto hpX = butter<2>((2.0f * f_low)  / fs);
auto lpX = butter<2>((2.0f * f_high) / fs);
auto hpY = butter<2>((2.0f * f_low)  / fs);
auto lpY = butter<2>((2.0f * f_high) / fs);
auto hpZ = butter<2>((2.0f * f_low)  / fs);
auto lpZ = butter<2>((2.0f * f_high) / fs);


// Bias offsets (deg/s)
float bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;


// Parameters
const int biasSamples = 200;
const float deadzone_threshold = 0.5f; // degrees/sec, below this treated as zero
const float freq_peak_ratio = 5.0f;    // dominant peak must be this times average to be trusted
const float min_peak_mag = 1e-3f;      // avoid reporting tiny noise as frequency


// Helper: compute RMS over full window
float calcRMS(const float arr[]) {
  float sumSq = 0.0f;
  for (int i = 0; i < windowSize; i++) sumSq += arr[i] * arr[i];
  return sqrt(sumSq / windowSize);
}


// Helper: find dominant frequency from magnitude spectrum with validation
float findDominantFreq(float vReal[], int len) {
  int half = len / 2;
  // compute average magnitude excluding DC (index 0)
  float sum = 0.0f;
  for (int i = 1; i < half; i++) sum += vReal[i];
  float avg = sum / (half - 1);
  int peakIndex = 1;
  float peakValue = vReal[1];
  for (int i = 2; i < half; i++) {
    if (vReal[i] > peakValue) {
      peakValue = vReal[i];
      peakIndex = i;
    }
  }
  // Validate peak: sufficiently above noise floor and above absolute floor
  if (peakValue < min_peak_mag) return 0.0f;
  if (avg > 0 && peakValue / avg < freq_peak_ratio) return 0.0f;
  return (peakIndex * fs) / len;
}


// Apply deadzone: zero small jitter
float applyDeadzone(float v) {
  if (fabs(v) < deadzone_threshold) return 0.0f;
  return v;
}


void calibrateGyroBias() {
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  for (int i = 0; i < biasSamples; i++) {
    int16_t ax, ay, az, gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);
    sum_gx += gx_raw;
    sum_gy += gy_raw;
    sum_gz += gz_raw;
    delay(10); // small spacing to spread over ~2 seconds total
  }
  // Convert to deg/s: scale factor for ±250dps is 131
  bias_gx = (sum_gx / (float)biasSamples) / 131.0f;
  bias_gy = (sum_gy / (float)biasSamples) / 131.0f;
  bias_gz = (sum_gz / (float)biasSamples) / 131.0f;
}


void primeFilters(int cycles=10) {
  // Run some dummy data to settle filter internal states
  for (int i = 0; i < cycles; i++) {
    float dummy = 0.0f;
    lpX(hpX(dummy));
    lpY(hpY(dummy));
    lpZ(hpZ(dummy));
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) delay(100);
  }


  // Calibration and filter priming
  Serial.println("Calibrating gyro bias...");
  calibrateGyroBias();
  primeFilters(20); // allow filter states to settle


  Serial.print("Biases (deg/s): gx="); Serial.print(bias_gx, 3);
  Serial.print(" gy="); Serial.print(bias_gy, 3);
  Serial.print(" gz="); Serial.println(bias_gz, 3);
  Serial.println("Starting measurement loop.");
}


void loop() {
  static unsigned long lastSampleMicros = 0;
  const unsigned long targetInterval = 1000000UL / fs; // microseconds per sample


  // Collect samples for FFT (per-axis) and update RMS per sample
  for (int i = 0; i < samples; i++) {
    // timing: wait to approximate consistent sampling interval
    unsigned long now = micros();
    if (lastSampleMicros != 0) {
      long delta = (long)(targetInterval - (now - lastSampleMicros));
      if (delta > 0) delayMicroseconds(delta);
    }
    lastSampleMicros = micros();


    int16_t ax, ay, az, gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);


    // Convert raw gyro to deg/s and subtract calibrated bias
    float gx = (gx_raw / 131.0f) - bias_gx;
    float gy = (gy_raw / 131.0f) - bias_gy;
    float gz = (gz_raw / 131.0f) - bias_gz;


    // Deadzone to suppress small noise around zero
    gx = applyDeadzone(gx);
    gy = applyDeadzone(gy);
    gz = applyDeadzone(gz);


    // Bandpass per axis (high-pass then low-pass)
    float gx_f = lpX(hpX(gx));
    float gy_f = lpY(hpY(gy));
    float gz_f = lpZ(hpZ(gz));


    // Update RMS sliding window
    gxWindow[rmsIndex] = gx_f;
    gyWindow[rmsIndex] = gy_f;
    gzWindow[rmsIndex] = gz_f;
    rmsIndex = (rmsIndex + 1) % windowSize;
    if (rmsIndex == 0) rmsFilled = true;


    // Prepare FFT inputs (imaginary zero)
    vRealX[i] = gx_f; vImagX[i] = 0.0f;
    vRealY[i] = gy_f; vImagY[i] = 0.0f;
    vRealZ[i] = gz_f; vImagZ[i] = 0.0f;
  }


  // Output RMS if window is filled
  if (rmsFilled) {
    Serial.print("RMS X: "); Serial.print(calcRMS(gxWindow), 2);
    Serial.print("°, Y: "); Serial.print(calcRMS(gyWindow), 2);
    Serial.print("°, Z: "); Serial.print(calcRMS(gzWindow), 2);
    Serial.println("°/s");
  }


  // FFT + dominant frequency per axis
  // X
  FFT.windowing(vRealX, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vRealX, vImagX, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vRealX, vImagX, samples);
  float freqX = findDominantFreq(vRealX, samples);


  // Y
  FFT.windowing(vRealY, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vRealY, vImagY, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vRealY, vImagY, samples);
  float freqY = findDominantFreq(vRealY, samples);


  // Z
  FFT.windowing(vRealZ, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vRealZ, vImagZ, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vRealZ, vImagZ, samples);
  float freqZ = findDominantFreq(vRealZ, samples);


  // Report dominant frequencies (0 means no significant peak)
  Serial.print("Dominant Freq X: ");
  if (freqX > 0.0f) Serial.print(freqX, 2); else Serial.print("n/a");
  Serial.print(" Hz, Y: ");
  if (freqY > 0.0f) Serial.print(freqY, 2); else Serial.print("n/a");
  Serial.print(" Hz, Z: ");
  if (freqZ > 0.0f) Serial.print(freqZ, 2); else Serial.print("n/a");
  Serial.println(" Hz");


  // small delay to separate reporting cycles (non-critical)
  delay(100);
}
