#include <Wire.h>
#include <MPU6050.h>  // Electronic Cats library


MPU6050 mpu;


// Sliding window parameters
const int windowSize = 50;
float gxWindow[windowSize] = {0};
float gyWindow[windowSize] = {0};
float gzWindow[windowSize] = {0};
int bufferIndex = 0;
bool bufferFilled = false;


void setup() {
  Serial.begin(115200);
  Wire.begin();


  mpu.initialize();


  // Set gyroscope range to ±250 deg/s (0x00)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);


  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }


  Serial.println("MPU6050 connected. Calculating RMS angular velocity in deg/s...");
}


void loop() {
  int16_t ax, ay, az;
  int16_t gx_raw, gy_raw, gz_raw;


  // Read raw data
  mpu.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);


  // Convert to deg/s using sensitivity scale factor
  float gx = gx_raw / 131.0;
  float gy = gy_raw / 131.0;
  float gz = gz_raw / 131.0;


  // Insert current sample into sliding window
  gxWindow[bufferIndex] = gx;
  gyWindow[bufferIndex] = gy;
  gzWindow[bufferIndex] = gz;
  bufferIndex = (bufferIndex + 1) % windowSize;


  if (bufferIndex == 0) bufferFilled = true;


  if (bufferFilled) {
    float rmsX = calcRMS(gxWindow);
    float rmsY = calcRMS(gyWindow);
    float rmsZ = calcRMS(gzWindow);


    Serial.print("RMS X: "); Serial.print(rmsX, 2); Serial.print(" °/s, ");
    Serial.print("Y: "); Serial.print(rmsY, 2); Serial.print(" °/s, ");
    Serial.print("Z: "); Serial.println(rmsZ, 2); Serial.print(" °/s");
  }


  delay(20);  // ~50 Hz
}


float calcRMS(float arr[]) {
  float sumSq = 0;
  for (int i = 0; i < windowSize; i++) {
    float val = arr[i];
    sumSq += val * val;
  }
  return sqrt(sumSq / windowSize);
}
