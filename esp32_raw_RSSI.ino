#define ESP_PLATFORM
#include <NimBLEDevice.h>
#include <math.h>
#include <esp_bt_main.h>
#include <esp_bt.h>

const bool USE_DEVICE_NAME = false;
const std::string TARGET_NAME = "RESCUE_BEACON";
const NimBLEUUID TARGET_UUID("4f2913e2-3489-4b68-b769-95213600f6ee");

#define ESP32_TX_TO_STM32_RX 17
#define ESP32_RX_FROM_STM32  16
#define UART_BAUD            115200

const int8_t RSSI_LOST_SENTINEL = -127;
const unsigned long LOST_TIMEOUT_MS = 500;

volatile int8_t g_latest_rssi = -100;
volatile bool   g_new_data    = false;
unsigned long   g_last_sample_ms = 0;

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    bool isMatch = false;
    if (USE_DEVICE_NAME && advertisedDevice->haveName() &&
        advertisedDevice->getName() == TARGET_NAME) {
      isMatch = true;
    }
    if (!USE_DEVICE_NAME && advertisedDevice->haveServiceUUID() &&
        advertisedDevice->isAdvertisingService(TARGET_UUID)) {
      isMatch = true;
    }
    if (isMatch) {
      g_latest_rssi = advertisedDevice->getRSSI();
      g_new_data = true;
    }
  }
};

const int MEDIAN_WINDOW = 5;
int8_t medianBuf[MEDIAN_WINDOW];
int medianIdx = 0;
int medianCount = 0;

int8_t medianFilter(int8_t newVal) {
  medianBuf[medianIdx] = newVal;
  medianIdx = (medianIdx + 1) % MEDIAN_WINDOW;
  if (medianCount < MEDIAN_WINDOW) medianCount++;

  int8_t sorted[MEDIAN_WINDOW];
  memcpy(sorted, medianBuf, medianCount * sizeof(int8_t));
  for (int i = 1; i < medianCount; i++) {
    int8_t key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
    sorted[j+1] = key;
  }
  return sorted[medianCount / 2];
}

double kf_estimate = -100.0;
double kf_error = 1.0;
const double KF_PROCESS_NOISE     = 0.6;
const double KF_MEASUREMENT_NOISE = 4.0;

double kalmanUpdate(double measurement) {
  kf_error += KF_PROCESS_NOISE;
  double gain = kf_error / (kf_error + KF_MEASUREMENT_NOISE);
  kf_estimate += gain * (measurement - kf_estimate);
  kf_error *= (1.0 - gain);
  return kf_estimate;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, ESP32_RX_FROM_STM32, ESP32_TX_TO_STM32_RX);

  NimBLEDevice::init("");
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new MyScanCallbacks(), true);
  pScan->setActiveScan(true);
  pScan->setInterval(60);
  pScan->setWindow(60);
  pScan->start(0, false);

  g_last_sample_ms = millis();
}

void loop() {
  if (g_new_data) {
    g_new_data = false;
    int8_t raw = g_latest_rssi;

    int8_t medianed = medianFilter(raw);
    double filtered = kalmanUpdate((double)medianed);
    g_last_sample_ms = millis();

    int rounded = (int)round(filtered);
    if (rounded > 127) rounded = 127;
    if (rounded < -126) rounded = -126;
    int8_t outByte = (int8_t)rounded;

    Serial2.write((uint8_t)outByte);

    Serial.print("raw:"); Serial.print(raw);
    Serial.print(" filt:"); Serial.print(filtered, 2);
    Serial.print(" sent:"); Serial.println(outByte);

  } else if (millis() - g_last_sample_ms > LOST_TIMEOUT_MS) {
    Serial2.write((uint8_t)RSSI_LOST_SENTINEL);
    Serial.println("target signal lost - sent sentinel byte");
    g_last_sample_ms = millis();
  }

  delay(20);
}
