#include <MPU9250.h>
#include "esp_adc_cal.h"
#include "charge.h"
#include <TFT_eSPI.h>
#include <pcf8563.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Free_Fonts.h"

#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager



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

MPU9250 mpu;
TFT_eSPI tft = TFT_eSPI();
PCF8563_Class rtc;
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
#define BLE_NAME "MM"  //must match filters name in bluetoothterminal.js- navigator.bluetooth.requestDevice
// BLEUUID  SERVICE_UUID((uint16_t)0x1802); // UART service UUID
// BLEUUID CHARACTERISTIC_UUID ((uint16_t)0x1803);

// define two tasks for Blink & AnalogRead
void TaskBluetooth(void *pvParameters);
void TaskReadMPU(void *pvParameters);

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

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("OTA Mode", tft.width() / 2, tft.height() / 2 - 20);
  //tft.drawString("configure wrist", tft.width()/2, tft.height() / 2 + 20);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(mac_address, tft.width() / 2, tft.height() / 2);
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor) {
  if (percentage == 0) {
    tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
  }
  uint8_t margin = 2;
  uint16_t barHeight = h - 2 * margin;
  uint16_t barWidth = w - 2 * margin;
  tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
  tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
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
              tft.fillScreen(TFT_BLACK);
              tft.drawString("Updating...", tft.width() / 2 - 20, 55);
            })
    .onEnd([]() {
      Serial.println("\nEnd");
      delay(500);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      int percentage = (progress / (total / 100));
      tft.setTextDatum(TC_DATUM);
      tft.setTextPadding(tft.textWidth(" 888% "));
      tft.drawString(String(percentage) + "%", 145, 35);
      drawProgressBar(10, 30, 120, 15, percentage, TFT_WHITE, TFT_ORANGE);
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");

      tft.fillScreen(TFT_BLACK);
      tft.drawString("Update Failed", tft.width() / 2 - 20, 55);
      delay(3000);
      otaStart = false;
      initial = 1;
      targetTime = millis() + 1000;
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
      omm = 99;
    });

  ArduinoOTA.begin();
}


String getVoltage() {
  uint16_t v = analogRead(BATT_ADC_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  return String(battery_voltage) + "V";
}

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

String Bone = "N/A";
bool calibrated = false;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    Serial.print("Received Value: ");
    Serial.println(value.c_str());


    if (value == "start") {
      if (!start) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(mac_address, tft.width() / 2, 10);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("CONNECTED", tft.width() / 2, tft.height() / 2);
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.drawString("Calibrate now", tft.width() / 2, tft.height() - 20);
      }
    } else if (value == "calibrate") {
      start = true;
    } else if (value == "dispoff") {
      if (calibrated) {
        digitalWrite(TFT_BL, LOW);
        tft.writecommand(ST7735_DISPOFF);
        displayOn = false;
      }

    } else if (value == "poweroff") {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("POWER OFF", tft.width() / 2, tft.height() / 2);
      // mpu.setSleepEnabled(true);
      mpu.sleep(true);
      Serial.println("Go to Sleep");
      delay(3000);
      tft.writecommand(ST7735_SLPIN);
      tft.writecommand(ST7735_DISPOFF);
      esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_deep_sleep_start();
    }

    else if (value == "restart") {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("POWER OFF", tft.width() / 2, tft.height() / 2);
      delay(3000);
      ESP.restart();
    }

    else if (value == "ota") {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Rebooting", tft.width() / 2, 30);
      tft.drawString("in OTA mode....", tft.width() / 2, 50);

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




  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  //tft.pushImage(0, 0, 160, 80, ttgo);


  tft.setFreeFont(FF17);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(random(0xFFFF), TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Mesquite Mocap", tft.width() / 2, 20);
  tft.drawString("https://mesquite.cc", tft.width() / 2, tft.height() - 40);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);


  pinMode(TP_PIN_PIN, INPUT);
  //! Must be set to pull-up output mode in order to wake up in deep sleep mode
  pinMode(TP_PWR_PIN, PULLUP);
  digitalWrite(TP_PWR_PIN, HIGH);

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

  Wire.begin();
  Wire.setClock(400000);
  delay(2000);

  setupADC();






  if (!mpu.setup(0x69)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }



  batt_level = getVoltage();

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(mac_address, tft.width() / 2, 10);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Connect at:", tft.width() / 2, 30);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("https://mesquite.cc", tft.width() / 2, 45);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);

  tft.drawString(batt_level, tft.width() / 2, tft.height() - 10);

  batt_level = ", " + getVoltage();

  while (!start) {
    if (digitalRead(TP_PIN_PIN) == HIGH) {
      if (!pressed) {
        pressed = true;
        pressedTime = millis();
      }

      if (millis() - pressedTime > 2000) {
        if (digitalRead(TP_PIN_PIN) == HIGH) {
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("POWER OFF", tft.width() / 2, tft.height() / 2);
          // mpu.setSleepEnabled(true);
          mpu.sleep(true);
          Serial.println("Go to Sleep");
          delay(3000);
          tft.writecommand(ST7735_SLPIN);
          tft.writecommand(ST7735_DISPOFF);
          esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
          esp_deep_sleep_start();

        } else {
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("Hold for 2 seconds", tft.width() / 2, tft.height() / 2);
        }
      }
    } else {
      pressed = false;
    }
    // delay(1000);
  }

  // calibrate anytime you want to
  //tft.drawString("Accel Gyro calibration - 5sec.",  20, tft.height() / 2 );
  Serial.println("Accel Gyro calibration will start in 5sec.");
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(mac_address, tft.width() / 2, 10);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("CALIBRATION (1)", tft.width() / 2, tft.height() / 2.5);

  tft.drawString("Keep flat and still..", tft.width() / 2, tft.height() / 1.5);
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();


  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(mac_address, tft.width() / 2, 10);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.drawString("CALIBRATION (2)", tft.width() / 2, tft.height() / 2.5);

  tft.drawString("Wave in 8 pattern", tft.width() / 2, tft.height() / 1.5);
  Serial.println("Please Wave device in a figure eight until done.");
  delay(2000);
  mpu.calibrateMag();

  mpu.verbose(false);

  calibrated = true;

  batt_level = ", " + getVoltage();


  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(mac_address, tft.width() / 2, 10);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Transmitting Data", tft.width() / 2, tft.height() / 2.5);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.drawString(Bone + batt_level, tft.width() / 2, tft.height() / 1.5);


  pinMode(TP_PIN_PIN, INPUT);
  //! Must be set to pull-up output mode in order to wake up in deep sleep mode
  pinMode(TP_PWR_PIN, PULLUP);
  digitalWrite(TP_PWR_PIN, HIGH);

  pinMode(LED_PIN, OUTPUT);

  pinMode(CHARGE_PIN, INPUT_PULLUP);

  attachInterrupt(
    CHARGE_PIN, [] {
      charge_indication = true;
    },
    CHANGE);

  if (digitalRead(CHARGE_PIN) == LOW) {
    charge_indication = true;
  }

  xTaskCreatePinnedToCore(
    TaskBluetooth, "TaskBluetooth"  // A name just for humans
    ,
    10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);

  xTaskCreatePinnedToCore(
    TaskReadMPU, "TaskReadMPU", 10000  // Stack size
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
  if (digitalRead(TP_PIN_PIN) == HIGH) {
    lastTouch = true;
    if (!pressed) {
      pressed = true;
      pressedTime = millis();
    }

    if (millis() - pressedTime > 4000) {
      if (digitalRead(TP_PIN_PIN) == HIGH) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("POWER OFF", tft.width() / 2, tft.height() / 2);
        // mpu.setSleepEnabled(true);
        mpu.sleep(true);
        Serial.println("Go to Sleep");
        delay(3000);
        tft.writecommand(ST7735_SLPIN);
        tft.writecommand(ST7735_DISPOFF);
        esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
        esp_deep_sleep_start();

      } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Hold for 2 seconds", tft.width() / 2, tft.height() / 2);
      }
    }
  } else {
    if (lastTouch) {
      tft.writecommand(ST7735_SLPIN);

      if (displayOn) {
        digitalWrite(TFT_BL, LOW);
        tft.writecommand(ST7735_DISPOFF);
        displayOn = false;

      } else {
        digitalWrite(TFT_BL, HIGH);
        tft.writecommand(ST7735_DISPON);
        batt_level = ", " + getVoltage();
        displayOn = true;
      }
    }
    pressed = false;
    lastTouch = false;
  }

  if (otaMode) {
    ArduinoOTA.handle();
  }

  //! If OTA starts, skip the following operation
  if (otaStart) {
    return;
  }
}

void IMU_Show() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 1000) {
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
    quat.x = mpu.getQuaternionX();
    quat.y = mpu.getQuaternionY();
    quat.z = mpu.getQuaternionZ();
    quat.w = mpu.getQuaternionW();
    // euler.x = mpu.getEulerX();
    // euler.y = mpu.getEulerY();
    // euler.z = mpu.getEulerZ();
  }
}

void print_roll_pitch_yaw() {
  //tft.setTextColor(TFT_GREEN, TFT_BLACK);
  //tft.fillScreen(TFT_BLACK);
  //tft.setTextDatum(TL_DATUM);
  // tft.drawString(mac_address,  0, 0, 0);
  //snprintf(buff, sizeof(buff), "%s", mac_address.c_str());
  //tft.drawString(buff, 0, 0);
  //snprintf(buff, sizeof(buff), "--  w   x   y   z");
  //tft.drawString(buff, 0, 8);
  //snprintf(buff, sizeof(buff), "Q %.2f  %.2f  %.2f  %.2f", quat.w, quat.x, quat.y, quat.z);
  //tft.drawString(buff, 0, 16);
  // snprintf(buff, sizeof(buff), "E %.2f  %.2f  %.2f", euler.x, euler.y, euler.z);
  // tft.drawString(buff, 0, 32);
  Serial.println(String(quat.w) + " " + String(quat.x) + " " + String(quat.y) + " " + String(quat.z));
}

void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
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

void TaskReadMPU(void *pvParameters) {
  for (;;) {
    IMU_Show();
    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
