#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_wifi.h>
#include <esp_now.h>

// These values determine the minimum and maximum pulse widths for your servos
// Do a sweep test to calibrate the correct values for your specific servo model
#define SERVOMIN  150  // Minimum value
#define SERVOMAX  600  // Maximum value
#define SERVOCENTER ((SERVOMIN + SERVOMAX) / 2)

#define TILT_SERVO  12 //Tilt Servo Motor on connector 12
#define PAN_SERVO  0 //Pan Servo Motor on connector 0

#define I2C_SDA 20
#define I2C_SCL 19

#define PIN_START_IN 2 // Pin to measure latency (input pulse from sender)
// Uncomment to enable latency logging
// #define ENABLE_LATENCY_LOG
#define ENABLE_DEBUG

// Smoothing params
const float SMOOTH_ALPHA = 1.0f;   // smoothing factor (0.0–1.0)
const int   DEADBAND     = 0;      // degrees, ignore small changes

// Interrupt handler for latency logging
volatile unsigned long t0 = 0;
void IRAM_ATTR onPulse() {
  t0 = micros(); // record start time when pulse arrived
}

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

struct YPRPacket {
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

YPRPacket receivedData;
volatile bool newData = false;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper function to read device MAC address
void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

// callback when ESP-NOW data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  newData = true;
}

void initPWM() {
  Serial.println("PCA9685 Initializing ...");
  pca9685.begin();
  pca9685.setPWMFreq(50); // 1 tick = 20 ms ÷ 4096 ≈ 4.88 µs
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Serial.begin(115200);
  while (!Serial) delay(10);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  initPWM();

#ifdef ENABLE_LATENCY_LOG
  pinMode(PIN_START_IN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_START_IN), onPulse, RISING);
#endif
}

void loop() {
  static float panFiltered = 90;
  static float tiltFiltered = 90;

  if(!newData) return;
  newData = false;

  // Convert received packet back to degrees
  float yaw   = receivedData.yaw / 100.0;   // [-180, 180] degrees
  float pitch = receivedData.pitch / 100.0; // [-90, 90] degrees
  float roll  = receivedData.roll / 100.0;  // [-180, 180] degrees

#if defined(ENABLE_DEBUG) && !defined(ENABLE_LATENCY_LOG)
  Serial.printf("Yaw: %.2f | Pitch: %.2f | Roll: %.2f\n", yaw, pitch, roll);
#endif

  int panAngle  = constrain(mapFloat(yaw,   -90.0, 90.0, 0.0, 180.0), 0, 180);
  int tiltAngle = constrain(mapFloat(pitch, -90.0, 90.0, 0.0, 180.0), 0, 180);

#ifdef ENABLE_LATENCY_LOG
  unsigned long t1 = micros();
  Serial.printf("%lu\n", t1 - t0);
#endif

  // Apply deadband + smoothing (currently not set)
  if (abs(panAngle - (int)panFiltered) > DEADBAND) {
    panFiltered = SMOOTH_ALPHA * panAngle + (1 - SMOOTH_ALPHA) * panFiltered;
    int pwmPan = map((int)panFiltered, 0, 180, SERVOMAX, SERVOMIN);
    pca9685.setPWM(PAN_SERVO, 0, pwmPan);
  }

  if (abs(tiltAngle - (int)tiltFiltered) > DEADBAND) {
    tiltFiltered = SMOOTH_ALPHA * tiltAngle + (1 - SMOOTH_ALPHA) * tiltFiltered;
    int pwmTilt = map((int)tiltFiltered, 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(TILT_SERVO, 0, pwmTilt);
  }

}
