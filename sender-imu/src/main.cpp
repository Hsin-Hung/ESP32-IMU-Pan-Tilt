#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#define I2C_SDA 19
#define I2C_SCL 20
#define INTERRUPT_PIN 2
#define PIN_START 26 // Pin to toggle for latency measurement, this should be connected to receiver board's PIN_START_IN pin

// uncomment to enable latency logging
// #define ENABLE_LATENCY_LOG
#define ENABLE_DEBUG

// receiver-servo MAC address, run readMacAddress() to get the address
uint8_t consumerMacAddress[] = {0xB0, 0x81, 0x84, 0x9B, 0x60, 0x28}; // TODO: replace with your receiver ESP32's MAC address

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
#ifdef ENABLE_LATENCY_LOG
  digitalWrite(PIN_START, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_START, LOW);
#endif
  mpuInterrupt = true;
}

struct YPRPacket {
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

esp_now_peer_info_t peerInfo;


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

// callback when ESP-NOW data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef ENABLE_DEBUG
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

void initMPU() {
  Serial.println(F("Initializing MPU..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

#ifdef ENABLE_DEBUG
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
#endif

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // You can supply your own acc and gyro offsets here after calibration
  // mpu.setXAccelOffset(1326);
  // mpu.setYAccelOffset(-1899);
  // mpu.setZAccelOffset(1258);

  // mpu.setXGyroOffset(-54);
  // mpu.setYGyroOffset(-29);
  // mpu.setZGyroOffset(31);

  if (devStatus == 0) {
    Serial.println(F("Please place the MPU6050 flat and still for calibration ..."));
    delay(3000); // give 3 seconds for the user to settle it
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    packetSize = mpu.dmpGetFIFOPacketSize();

    dmpReady = true;
    Serial.println(F("DMP ready!"));
  } else {
    Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
  }

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

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, consumerMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  initMPU();

#ifdef ENABLE_LATENCY_LOG
  pinMode(PIN_START, OUTPUT);
  digitalWrite(PIN_START, LOW);
#endif
}

void loop() {

  if (!dmpReady) return;

  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw  = ypr[0] * RAD_TO_DEG;
    float pitch = ypr[1] * RAD_TO_DEG;
    float roll  = ypr[2] * RAD_TO_DEG;

#ifdef ENABLE_DEBUG
    Serial.printf("Yaw: %.2f | Pitch: %.2f | Roll: %.2f\n", yaw, pitch, roll);
#endif

    // Build packet
    YPRPacket pkt = {
      (int16_t)(yaw * 100.0f),
      (int16_t)(pitch * 100.0f),
      (int16_t)(roll * 100.0f)
    };

    if (esp_now_send(consumerMacAddress, (uint8_t *) &pkt, sizeof(pkt)) != ESP_OK) {
      Serial.println("Error sending the data");
    }
  }
}
