#include "esp_adc_cal.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

String response;

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1

unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 1000 * 60 * 2;

String batt_level = "";
bool displayOn = true;
bool lastTouch = false;

bool otaMode = false;

int writeCount = 0;

boolean start = false;

BLECharacteristic *pCharacteristic;

struct Quat {
  float x;
  float y;
  float z;
  float w;
} quat;
struct Euler {
  float x;
  float y;
  float z;
} euler;
char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;

uint8_t func_select = 0;
uint8_t omm = 99;
uint8_t xcolon = 0;
uint32_t targetTime = 0;  // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;

uint8_t hh, mm, ss;
String mac_address;
int pacnum = 0;

#define TP_PIN_PIN 33
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define IMU_INT_PIN 38
#define RTC_INT_PIN 34
#define BATT_ADC_PIN 35
#define VBUS_PIN 36
#define TP_PWR_PIN 25
#define LED_PIN 4
#define CHARGE_PIN 32

//maintain compatability with HM-10
#define BLE_NAME "WebXCube"  //must match filters name in bluetoothterminal.js- navigator.bluetooth.requestDevice
// BLEUUID  SERVICE_UUID((uint16_t)0x1802); // UART service UUID
// BLEUUID CHARACTERISTIC_UUID ((uint16_t)0x1803);

// define two tasks for Blink & AnalogRead
void TaskBluetooth(void *pvParameters);
void TaskReadBNO(void *pvParameters);

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");  // UART service UUID
BLEUUID CHARACTERISTIC_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEAdvertising *pAdvertising;

void setupWiFi() {
  WiFiManager wifiManager;
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setBreakAfterConfig(true);  // Without this saveConfigCallback does not get fired
  mac_address = WiFi.macAddress();
  Serial.println(mac_address);
  wifiManager.autoConnect(String("MM-" + mac_address).c_str());
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor) {

}

void setupOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(mac_address.c_str());

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
              String type;
              if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
              else  // U_SPIFFS
                type = "filesystem";

              // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
              Serial.println("Start updating " + type);
              otaStart = true;
            })
    .onEnd([]() {
      Serial.println("\nEnd");
      delay(500);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      int percentage = (progress / (total / 100));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");

      delay(3000);
      otaStart = false;
      initial = 1;
      targetTime = millis() + 1000;
      omm = 99;
    });

  ArduinoOTA.begin();
}

String getVoltage() {
  uint16_t v = analogRead(BATT_ADC_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  return String(battery_voltage) + "V";
}

/*
void setupADC() {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    Serial.println("Default Vref: 1100mV");
  }
}
*/

String Bone = "N/A";
bool calibrated = false;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    Serial.print("Received Value: ");
    Serial.println(value.c_str());

    if (value == "start") {
      if (!start) {
      }
    } else if (value == "calibrate") {
      start = true;
    } else if (value == "dispoff") {
      if (calibrated) {

        displayOn = false;
      }

    }
    /* 
    else if (value == "poweroff") {

      esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_deep_sleep_start();
    }

*/
    else if (value == "restart") {
      delay(3000);
      ESP.restart();
    }

    else if (value == "ota") {
      EEPROM.write(0, 1);
      EEPROM.commit();
      delay(2000);
      ESP.restart();
    } else {
      Bone = String(value.c_str());
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer *server) {
    Serial.print("Disconnected");
    pAdvertising->start();
  }
};

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(LED_PIN, OUTPUT);

  int otaState = EEPROM.read(0);
  Serial.println(otaState);

  if (otaState == 1) {
    EEPROM.write(0, 0);
    EEPROM.commit();
    otaMode = true;
    otaStart = true;
    setupWiFi();
    setupOTA();
    return;
  }

  BLEDevice::init(BLE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->addDescriptor(new BLE2902());

  mac_address = BLEDevice::getAddress().toString().c_str();

  esp_ble_gap_set_device_name(("MM-" + mac_address).c_str());
  esp_bt_dev_set_device_name(("MM-" + mac_address).c_str());

  pService->start();

  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  
  myIMU.begin(0x69, Wire);

  Wire.begin(9, 8);
  Wire.setClock(400000);
  delay(2000);

  // setupADC();

  delay(100); //  Wait for BNO to boot
  // Start i2c and BNO080
  Wire.flush();   // Reset I2C
  //myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
  //myIMU.begin(0x69, Wire);

  //Wire.begin(9, 8);

   if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

  xTaskCreatePinnedToCore(
    TaskBluetooth, "TaskBluetooth"  // A name just for humans
    ,
    10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);

  xTaskCreatePinnedToCore(
    TaskReadBNO, "TaskReadBNO", 10000  // Stack size
    ,
    NULL, 1  // Priority
    ,
    NULL, 1);
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    batt_level = ", " + getVoltage();
    lastTime = millis();
  }
  Serial.println(pressed);
  Serial.println(millis());

}

void IMU_Show() {
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    Serial.print(quatI, 2);
    Serial.print(F(" "));
    Serial.print(quatK, 2);
    Serial.print(F(" "));
    Serial.print(quatJ, 2);
    Serial.print(F(" "));
    Serial.println(quatReal, 2);

  }
}

void print_calibration() {

}

void TaskBluetooth(void *pvParameters) {
  for (;;) {

    static uint32_t prev_ms_ble = millis();
    if (millis() > prev_ms_ble + 1000 / 40) {
      prev_ms_ble = millis();
      String url = mac_address + " " + quat.x + " " + quat.y + " " + quat.z + " " + quat.w;
      pCharacteristic->setValue(url.c_str());
      pCharacteristic->notify();
      vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
    }
  }
}

void TaskReadBNO(void *pvParameters) {
  for (;;) {
    IMU_Show();
    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
