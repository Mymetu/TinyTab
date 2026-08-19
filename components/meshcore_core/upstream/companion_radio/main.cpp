#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#ifdef TINYLORA_C3_COMPANION_IDLE_DELAY_MS
  #include <esp_pm.h>
#endif

#include "MyMesh.h"

#if defined(ESP32)
  #include <esp_partition.h>
  #include <esp_spiffs.h>

#if (GPS_HISTORY_MAX_RECORDS > 0 && ENV_INCLUDE_GPS == 1) || \
    (REMOTE_GPS_HISTORY == 1 && REMOTE_GPS_HISTORY_MAX_RECORDS > 0)
static bool isSpiffsPartitionBlank() {
  const esp_partition_t* partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
  if (!partition) return false;

  uint8_t buffer[256];
  for (size_t offset = 0; offset < partition->size; offset += sizeof(buffer)) {
    size_t remaining = (size_t)(partition->size - offset);
    size_t read_size = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
    if (esp_partition_read(partition, offset, buffer, read_size) != ESP_OK) return false;
    for (size_t i = 0; i < read_size; i++) {
      if (buffer[i] != 0xFF) return false;
    }
  }
  return true;
}
#endif

static bool mountSpiffs(bool preserve_existing_data) {
  if (SPIFFS.begin(false)) return true;

#if (GPS_HISTORY_MAX_RECORDS > 0 && ENV_INCLUDE_GPS == 1) || \
    (REMOTE_GPS_HISTORY == 1 && REMOTE_GPS_HISTORY_MAX_RECORDS > 0)
  if (preserve_existing_data && !isSpiffsPartitionBlank()) {
    Serial.println("ERROR: SPIFFS contains data but cannot be mounted; format skipped");
    return false;
  }
#else
  (void)preserve_existing_data;
#endif

  // The PM framework's Arduino SPIFFS.format() wrapper tries to remove an
  // unregistered idle task from the watchdog. Format through ESP-IDF instead.
  Serial.println("INFO: SPIFFS mount failed; formatting filesystem...");
  esp_err_t format_result = esp_spiffs_format(nullptr);
  if (format_result != ESP_OK) {
    Serial.printf("ERROR: SPIFFS format failed: %d\n", (int)format_result);
    return false;
  }

  Serial.println("INFO: SPIFFS format complete; mounting filesystem...");
  if (!SPIFFS.begin(false)) {
    Serial.println("ERROR: SPIFFS mount failed after format");
    return false;
  }
  Serial.println("INFO: SPIFFS mounted successfully");
  return true;
}
#endif

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
  #if defined(QSPIFLASH)
    #include <CustomLFS_QSPIFlash.h>
    DataStore store(InternalFS, QSPIFlash, rtc_clock);
  #else
  #if defined(EXTRAFS)
    #include <CustomLFS.h>
    CustomLFS ExtraFS(0xD4000, 0x19000, 128);
    DataStore store(InternalFS, ExtraFS, rtc_clock);
  #else
    DataStore store(InternalFS, rtc_clock);
  #endif
  #endif
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
  DataStore store(LittleFS, rtc_clock);
#elif defined(ESP32)
  #include <SPIFFS.h>
  DataStore store(SPIFFS, rtc_clock);
#endif

#ifdef ESP32
  #ifdef WIFI_SSID
    #include <helpers/esp32/SerialWifiInterface.h>
    SerialWifiInterface serial_interface;
    #ifndef TCP_PORT
      #define TCP_PORT 5000
    #endif
  #elif defined(BLE_PIN_CODE)
    #include <helpers/esp32/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #elif defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(RP2040_PLATFORM)
  //#ifdef WIFI_SSID
  //  #include <helpers/rp2040/SerialWifiInterface.h>
  //  SerialWifiInterface serial_interface;
  //  #ifndef TCP_PORT
  //    #define TCP_PORT 5000
  //  #endif
  // #elif defined(BLE_PIN_CODE)
  //   #include <helpers/rp2040/SerialBLEInterface.h>
  //   SerialBLEInterface serial_interface;
  #if defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(NRF52_PLATFORM)
  #ifdef BLE_PIN_CODE
    #include <helpers/nrf52/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(STM32_PLATFORM)
  #include <helpers/ArduinoSerialInterface.h>
  ArduinoSerialInterface serial_interface;
#else
  #error "need to define a serial interface"
#endif

/* GLOBAL OBJECTS */
#ifdef DISPLAY_CLASS
  #include "UITask.h"
  UITask ui_task(&board, &serial_interface);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables, store
   #ifdef DISPLAY_CLASS
      , &ui_task
   #endif
);

/* END GLOBAL OBJECTS */

void halt() {
  while (1) ;
}

/* WIFI RECONNECT TRACKERS */
#if defined(ESP32) && defined(WIFI_SSID)
  bool wifi_needs_reconnect = false;
  unsigned long last_wifi_reconnect_attempt = 0;
#endif

void setup() {
  Serial.begin(115200);

  board.begin();

#ifdef TINYLORA_C3_COMPANION_IDLE_DELAY_MS
  static esp_pm_config_esp32c3_t pm_config;
  pm_config.max_freq_mhz = F_CPU / 1000000;
  pm_config.min_freq_mhz = 20;
  pm_config.light_sleep_enable = false;
  esp_err_t pm_result = esp_pm_configure(&pm_config);
  MESH_DEBUG_PRINTLN("C3 companion power management config result: %d", (int)pm_result);
#endif

#ifdef DISPLAY_CLASS
  DisplayDriver* disp = NULL;
  if (display.begin()) {
    disp = &display;
    disp->startFrame();
    disp->setTextSize(1);
    disp->drawTextCentered(disp->width() / 2, 16, "Initializing...");
    disp->drawTextCentered(disp->width() / 2, 36, u8"初期化中");
    disp->endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  #if defined(QSPIFLASH)
    if (!QSPIFlash.begin()) {
      // debug output might not be available at this point, might be too early. maybe should fall back to InternalFS here?
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: failed to initialize");
    } else {
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: initialized successfully");
    }
  #else
  #if defined(EXTRAFS)
      ExtraFS.begin();
  #endif
  #endif
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef BLE_PIN_CODE
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

  //#ifdef WIFI_SSID
  //  WiFi.begin(WIFI_SSID, WIFI_PWD);
  //  serial_interface.begin(TCP_PORT);
  // #elif defined(BLE_PIN_CODE)
  //   char dev_name[32+16];
  //   sprintf(dev_name, "%s%s", BLE_NAME_PREFIX, the_mesh.getNodeName());
  //   serial_interface.begin(dev_name, the_mesh.getBLEPin());
  #if defined(SERIAL_RX)
    companion_serial.setPins(SERIAL_RX, SERIAL_TX);
    companion_serial.begin(115200);
    serial_interface.begin(companion_serial);
  #else
    serial_interface.begin(Serial);
  #endif
    the_mesh.startInterface(serial_interface);
#elif defined(ESP32)
#if (GPS_HISTORY_MAX_RECORDS > 0 && ENV_INCLUDE_GPS == 1) || \
    (REMOTE_GPS_HISTORY == 1 && REMOTE_GPS_HISTORY_MAX_RECORDS > 0)
  if (!mountSpiffs(true)) {
    Serial.println("ERROR: SPIFFS mount failed; automatic format disabled to protect stored data");
  }
#else
  if (!mountSpiffs(false)) {
    Serial.println("ERROR: SPIFFS unavailable; settings cannot be persisted");
  }
#endif
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef WIFI_SSID
  board.setInhibitSleep(true);   // prevent sleep when WiFi is active
  WiFi.setAutoReconnect(true);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
      if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
          WIFI_DEBUG_PRINTLN("WiFi disconnected. Flagging for reconnect...");
          wifi_needs_reconnect = true;
      } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
          WIFI_DEBUG_PRINTLN("WiFi connected successfully!");
          wifi_needs_reconnect = false;
      }
  });

  WiFi.begin(WIFI_SSID, WIFI_PWD);
  serial_interface.begin(TCP_PORT);
#elif defined(BLE_PIN_CODE)
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#elif defined(SERIAL_RX)
  companion_serial.setPins(SERIAL_RX, SERIAL_TX);
  companion_serial.begin(115200);
  serial_interface.begin(companion_serial);
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#else
  #error "need to define filesystem"
#endif

  sensors.begin();

#if ENV_INCLUDE_GPS == 1
  the_mesh.applyGpsPrefs();
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(disp, &sensors, the_mesh.getNodePrefs());  // still want to pass this in as dependency, as prefs might be moved
#endif

  board.onBootComplete();
}

void loop() {
  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  if (!the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#endif
  }

#if defined(ESP32) && defined(WIFI_SSID)
  // Safely attempt to reconnect every 10 seconds if flagged
  if (wifi_needs_reconnect && (millis() - last_wifi_reconnect_attempt > 10000)) {
    WIFI_DEBUG_PRINTLN("Attempting manual WiFi reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    last_wifi_reconnect_attempt = millis();
  }
#endif

#ifdef TINYLORA_C3_COMPANION_IDLE_DELAY_MS
  // BLE controller tasks wake independently; this only bounds application polling latency.
  delay(TINYLORA_C3_COMPANION_IDLE_DELAY_MS);
#endif
}
