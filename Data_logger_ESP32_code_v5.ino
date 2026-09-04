#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ELT0169 TX connects to GPIO25 (ESP32 RX).
// ELT0169 RX connects to GPIO26 (ESP32 TX).
constexpr int GPS_RX_PIN = 25;
constexpr int GPS_TX_PIN = 26;
constexpr uint32_t GPS_BAUD = 9600;

// VESC COMM UART.
// VESC TX connects to GPIO16 (ESP32 RX).
// VESC RX connects to GPIO17 (ESP32 TX).
constexpr int VESC_RX_PIN = 16;
constexpr int VESC_TX_PIN = 17;
constexpr uint32_t VESC_BAUD = 115200;
constexpr uint8_t COMM_GET_VALUES = 4;

// microSD module using the ESP32 VSPI pins.
constexpr int SD_CS_PIN = 5;
constexpr int SD_SCK_PIN = 18;
constexpr int SD_MISO_PIN = 19;
constexpr int SD_MOSI_PIN = 23;
constexpr uint32_t SD_SPI_FREQ = 1000000;
const char VESC_CSV_HEADER[] =
    "ms_today;input_voltage;temp_mos_max;temp_mos_1;temp_mos_2;temp_mos_3;"
    "temp_motor;current_motor;current_in;d_axis_current;q_axis_current;erpm;"
    "duty_cycle;amp_hours_used;amp_hours_charged;watt_hours_used;"
    "watt_hours_charged;tachometer;tachometer_abs;encoder_position;fault_code;"
    "vesc_id;d_axis_voltage;q_axis_voltage;ms_today_setup;amp_hours_setup;"
    "amp_hours_charged_setup;watt_hours_setup;watt_hours_charged_setup;"
    "battery_level;battery_wh_tot;current_in_setup;current_motor_setup;"
    "speed_meters_per_sec;tacho_meters;tacho_abs_meters;num_vescs;"
    "ms_today_imu;roll;pitch;yaw;accX;accY;accZ;gyroX;gyroY;gyroZ;"
    "gnss_posTime;gnss_lat;gnss_lon;gnss_alt;gnss_gVel;gnss_vVel;"
    "gnss_hAcc;gnss_vAcc;";
const char WIFI_SSID[] = "eFoilLogger";
const char WIFI_PASSWORD[] = "efoillogger";
constexpr byte DNS_PORT = 53;

// External status LED on GPIO27.
constexpr int LED_PIN = 27;
constexpr bool LED_ON = HIGH;
constexpr bool LED_OFF = LOW;

constexpr uint32_t REPORT_INTERVAL_MS = 1000;
constexpr bool SERIAL_LIVE_STATUS = false;
constexpr uint32_t SD_RETRY_INTERVAL_MS = 5000;
constexpr uint32_t VESC_POLL_INTERVAL_MS = 100;
constexpr uint32_t LOG_INTERVAL_MS = 200;
constexpr uint32_t LOG_FLUSH_INTERVAL_MS = 1000;
constexpr uint32_t LOG_CHECKPOINT_INTERVAL_MS = 30000;
constexpr bool LOG_FLUSH_EVERY_ROW = true;
constexpr uint32_t VESC_MAX_AGE_MS = 1000;
constexpr uint32_t FIX_MAX_AGE_MS = 2000;
constexpr uint32_t LED_GNSS_WAIT_BLINK_INTERVAL_MS = 250;
constexpr uint32_t LED_RECORD_PULSE_INTERVAL_MS = 2000;
constexpr uint32_t LED_RECORD_PULSE_OFF_MS = 120;
constexpr float START_SPEED_MPS = 1.0f;
constexpr float START_ERPM_ABS = 300.0f;
constexpr uint32_t START_CONFIRM_MS = 3000;
constexpr float STOP_SPEED_MPS = 0.7f;
constexpr float STOP_ERPM_ABS = 200.0f;
constexpr float STOP_INPUT_CURRENT_A = 1.0f;
constexpr uint32_t STOP_SPEED_SAMPLE_INTERVAL_MS = 1000;
constexpr uint8_t STOP_SPEED_WINDOW_SAMPLES = 60;
constexpr uint8_t STOP_SPEED_LOW_SAMPLES_REQUIRED = 54;
constexpr bool AUTO_STOP_ENABLED = true;
constexpr float DEFAULT_GNSS_HACC_M = 3.0f;
constexpr float DEFAULT_GNSS_VACC_M = 5.0f;
constexpr size_t MAX_WEB_FILES = 40;
constexpr uint8_t VESC_CSV_SEMICOLONS = 55;
constexpr uint8_t WEB_REFRESH_SECONDS = 5;
constexpr int32_t LOG_TIME_OFFSET_SECONDS = 2 * 3600;
constexpr uint32_t MS_PER_DAY = 86400000UL;

enum LoggerState {
  STATE_IDLE,
  STATE_RECORDING
};

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial vescSerial(2);
WebServer *server = nullptr;
DNSServer dnsServer;

uint32_t lastReportMs = 0;
uint32_t lastSdRetryMs = 0;
uint32_t lastVescPollMs = 0;
uint32_t lastVescDataMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastLogFlushMs = 0;
uint32_t lastLogCheckpointMs = 0;
uint32_t lastLedToggleMs = 0;
uint32_t logStartMillis = 0;
uint32_t logStartMsToday = 0;
uint32_t recordingStartedMillis = 0;
uint32_t lastRecordingDurationMs = 0;
uint32_t skippedBadRows = 0;
uint32_t loggedRows = 0;
uint32_t lastKnownLogSize = 0;
bool ledState = false;
bool sdReady = false;
bool logReady = false;
bool gnss5HzConfigured = false;
bool manualStartPending = false;
LoggerState loggerState = STATE_IDLE;
char logFileName[32] = "";
char lastStopReason[40] = "none";
File logFile;
uint32_t speedStartSinceMs = 0;
uint32_t erpmStartSinceMs = 0;
uint32_t lastStopSpeedSampleMs = 0;
uint8_t stopSpeedSampleIndex = 0;
uint8_t stopSpeedSampleCount = 0;
uint8_t stopSpeedLowSampleCount = 0;
bool stopSpeedLowSamples[STOP_SPEED_WINDOW_SAMPLES] = {false};
float vescTempMos = 0.0f;
float vescTempMotor = 0.0f;
float vescMotorCurrent = 0.0f;
float vescInputCurrent = 0.0f;
float vescDuty = 0.0f;
float vescRpm = 0.0f;
float vescVoltage = 0.0f;
float vescAmpHours = 0.0f;
float vescAmpHoursCharged = 0.0f;
float vescWattHours = 0.0f;
float vescWattHoursCharged = 0.0f;
int32_t vescTachometer = 0;
int32_t vescTachometerAbs = 0;
uint8_t vescFaultCode = 0;

void printTwoDigits(uint8_t value) {
  if (value < 10) {
    Serial.print('0');
  }
  Serial.print(value);
}

void printStatus() {
  const bool hasFix =
      gps.location.isValid() && gps.location.age() <= FIX_MAX_AGE_MS;

  Serial.print(F("GNSS fix="));
  Serial.print(hasFix ? F("YES") : F("NO"));

  Serial.print(F(", state="));
  Serial.print(loggerState == STATE_RECORDING ? F("RECORDING") : F("IDLE"));
  if (manualStartPending) {
    Serial.print(F("(manual start pending)"));
  }

  Serial.print(F(", sats="));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("?"));
  }

  Serial.print(F(", hdop="));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 1);
  } else {
    Serial.print(F("?"));
  }

  Serial.print(F(", pos="));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 7);
    Serial.print(F("/"));
    Serial.print(gps.location.lng(), 7);
  } else {
    Serial.print(F("waiting"));
  }

  Serial.print(F(", speed="));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 2);
    Serial.print(F("km/h"));
  } else {
    Serial.print(F("?"));
  }

  Serial.print(F(", local="));
  if (gps.date.isValid() && gps.time.isValid()) {
    uint16_t localYear;
    uint8_t localMonth;
    uint8_t localDay;
    uint8_t localHour;
    uint8_t localMinute;
    uint8_t localSecond;
    localGpsDateTime(localYear, localMonth, localDay,
                     localHour, localMinute, localSecond);

    Serial.print(localYear);
    Serial.print('-');
    printTwoDigits(localMonth);
    Serial.print('-');
    printTwoDigits(localDay);
    Serial.print(' ');
    printTwoDigits(localHour);
    Serial.print(':');
    printTwoDigits(localMinute);
    Serial.print(':');
    printTwoDigits(localSecond);
  } else {
    Serial.print(F("waiting"));
  }

  Serial.print(F(", nmea="));
  Serial.print(gps.charsProcessed());

  Serial.print(F(", VESC="));
  if (millis() - lastVescDataMs <= VESC_MAX_AGE_MS) {
    Serial.print(F("duty="));
    Serial.print(vescDuty * 100.0f, 1);
    Serial.print(F("% erpm="));
    Serial.print(vescRpm, 0);
    Serial.print(F(" motorA="));
    Serial.print(vescMotorCurrent, 1);
    Serial.print(F(" inputA="));
    Serial.print(vescInputCurrent, 1);
    Serial.print(F(" volts="));
    Serial.print(vescVoltage, 1);
  } else {
    Serial.print(F("waiting"));
  }

  Serial.print(F(", SD="));
  Serial.print(sdReady ? F("OK") : F("not ready"));

  Serial.print(F(", log="));
  if (logReady) {
    Serial.print(logFileName);
    Serial.print(F(", rows="));
    Serial.print(loggedRows);
    Serial.print(F(", size="));
    Serial.println(lastKnownLogSize);
  } else if (loggerState == STATE_IDLE && !manualStartPending) {
    Serial.println(F("not recording"));
  } else if (!sdReady) {
    Serial.println(F("waiting for SD"));
  } else if (!gpsDateTimeIsSane()) {
    Serial.println(F("waiting for valid GNSS date/time"));
  } else if (!gpsLocationIsUsable()) {
    Serial.println(F("waiting for usable GNSS position"));
  } else {
    Serial.println(F("ready"));
  }
}

uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0;

  for (uint16_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

int16_t readInt16(const uint8_t *buffer, uint16_t &index) {
  uint16_t raw = (static_cast<uint16_t>(buffer[index]) << 8) |
                 static_cast<uint16_t>(buffer[index + 1]);
  index += 2;
  return static_cast<int16_t>(raw);
}

int32_t readInt32(const uint8_t *buffer, uint16_t &index) {
  uint32_t raw = (static_cast<uint32_t>(buffer[index]) << 24) |
                 (static_cast<uint32_t>(buffer[index + 1]) << 16) |
                 (static_cast<uint32_t>(buffer[index + 2]) << 8) |
                 static_cast<uint32_t>(buffer[index + 3]);
  index += 4;
  return static_cast<int32_t>(raw);
}

void sendVescGetValues() {
  const uint8_t payload[] = {COMM_GET_VALUES};
  const uint16_t crc = crc16(payload, sizeof(payload));

  vescSerial.write(2);
  vescSerial.write(sizeof(payload));
  vescSerial.write(payload, sizeof(payload));
  vescSerial.write(static_cast<uint8_t>(crc >> 8));
  vescSerial.write(static_cast<uint8_t>(crc & 0xFF));
  vescSerial.write(3);
}

void parseVescPayload(const uint8_t *payload, uint16_t len) {
  if (len < 1 || payload[0] != COMM_GET_VALUES) {
    return;
  }

  // COMM_GET_VALUES response layout starts with temperatures/currents, then
  // duty cycle and ERPM. Skip fields we are not showing in this test sketch.
  uint16_t index = 1;
  if (len < 55) {
    return;
  }

  vescTempMos = readInt16(payload, index) / 10.0f;
  vescTempMotor = readInt16(payload, index) / 10.0f;
  vescMotorCurrent = readInt32(payload, index) / 100.0f;
  vescInputCurrent = readInt32(payload, index) / 100.0f;
  readInt32(payload, index);                 // id current / 100
  readInt32(payload, index);                 // iq current / 100
  vescDuty = readInt16(payload, index) / 1000.0f;
  vescRpm = readInt32(payload, index);
  vescVoltage = readInt16(payload, index) / 10.0f;
  vescAmpHours = readInt32(payload, index) / 10000.0f;
  vescAmpHoursCharged = readInt32(payload, index) / 10000.0f;
  vescWattHours = readInt32(payload, index) / 10000.0f;
  vescWattHoursCharged = readInt32(payload, index) / 10000.0f;
  vescTachometer = readInt32(payload, index);
  vescTachometerAbs = readInt32(payload, index);
  vescFaultCode = payload[index];
  lastVescDataMs = millis();
}

void pollVesc(uint32_t now) {
  if (now - lastVescPollMs < VESC_POLL_INTERVAL_MS) {
    return;
  }

  lastVescPollMs = now;
  sendVescGetValues();
}

void readVesc() {
  static uint8_t packet[256];
  static uint16_t index = 0;
  static uint16_t payloadLen = 0;
  static uint8_t crcHigh = 0;
  static uint8_t crcLow = 0;
  static uint8_t state = 0;

  while (vescSerial.available() > 0) {
    const uint8_t value = vescSerial.read();

    switch (state) {
      case 0:
        if (value == 2) {
          state = 1;
          index = 0;
          payloadLen = 0;
        }
        break;

      case 1:
        payloadLen = value;
        if (payloadLen == 0 || payloadLen > sizeof(packet)) {
          state = 0;
        } else {
          state = 2;
        }
        break;

      case 2:
        packet[index++] = value;
        if (index >= payloadLen) {
          state = 3;
        }
        break;

      case 3: {
        crcHigh = value;
        state = 4;
        break;
      }

      case 4:
        crcLow = value;
        state = 5;
        break;

      case 5: {
        const uint8_t endByte = value;
        const uint16_t receivedCrc =
            (static_cast<uint16_t>(crcHigh) << 8) | crcLow;

        if (endByte == 3 && receivedCrc == crc16(packet, payloadLen)) {
          parseVescPayload(packet, payloadLen);
        }

        state = 0;
        break;
      }
    }
  }
}

void testSdCard() {
  Serial.println();
  Serial.println(F("--- SD card test ---"));
  Serial.println(F("Pins: CS=5, SCK=18, MISO=19, MOSI=23"));

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQ)) {
    Serial.println(F("SD init failed. Check wiring, power, card format and CS pin."));
    sdReady = false;
    return;
  }

  sdReady = true;
  Serial.println(F("SD init OK"));

  uint64_t cardSizeMb = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.print(F("Card size: "));
  Serial.print(cardSizeMb);
  Serial.println(F(" MB"));
  Serial.println(F("--- SD init done ---"));
}

bool ensureSdReady(bool verbose = false) {
  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    File root = SD.open("/");
    if (root) {
      root.close();
      sdReady = true;
      return true;
    }

    delay(100);
  }

  if (verbose) {
    Serial.println(F("SD root not available."));
    Serial.println(F("Trying automatic SD remount..."));
  }

  SD.end();
  delay(300);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdReady = SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQ);

  if (sdReady) {
    File root = SD.open("/");
    if (root) {
      root.close();
      if (verbose) {
        Serial.println(F("Automatic SD remount OK"));
      }
      return true;
    }
  }

  sdReady = false;
  if (verbose) {
    Serial.println(F("Automatic SD remount failed."));
  }
  return false;
}

bool remountSd(bool verbose = false) {
  if (verbose) {
    Serial.println(F("Manual SD remount..."));
  }

  SD.end();
  delay(300);
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdReady = SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQ);

  if (sdReady) {
    File root = SD.open("/");
    if (root) {
      root.close();
    } else {
      sdReady = false;
    }
  }

  if (verbose) {
    Serial.println(sdReady ? F("Manual SD remount OK") : F("Manual SD remount failed"));
  }

  return sdReady;
}

uint32_t gpsMsToday() {
  if (!gps.time.isValid()) {
    return millis();
  }

  return static_cast<uint32_t>(gps.time.hour()) * 3600000UL +
         static_cast<uint32_t>(gps.time.minute()) * 60000UL +
         static_cast<uint32_t>(gps.time.second()) * 1000UL +
         static_cast<uint32_t>(gps.time.centisecond()) * 10UL;
}

bool isLeapYear(uint16_t year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t days[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  if (month == 2 && isLeapYear(year)) {
    return 29;
  }

  if (month < 1 || month > 12) {
    return 31;
  }

  return days[month - 1];
}

void incrementDate(uint16_t &year, uint8_t &month, uint8_t &day) {
  day++;
  if (day <= daysInMonth(year, month)) {
    return;
  }

  day = 1;
  month++;
  if (month <= 12) {
    return;
  }

  month = 1;
  year++;
}

void localGpsDateTime(uint16_t &year, uint8_t &month, uint8_t &day,
                      uint8_t &hour, uint8_t &minute, uint8_t &second) {
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();

  int32_t localSeconds =
      static_cast<int32_t>(gps.time.hour()) * 3600L +
      static_cast<int32_t>(gps.time.minute()) * 60L +
      static_cast<int32_t>(gps.time.second()) +
      LOG_TIME_OFFSET_SECONDS;

  while (localSeconds >= 86400L) {
    localSeconds -= 86400L;
    incrementDate(year, month, day);
  }

  hour = localSeconds / 3600L;
  minute = (localSeconds % 3600L) / 60L;
  second = localSeconds % 60L;
}

uint32_t gpsLocalMsToday() {
  if (!gps.time.isValid()) {
    return millis();
  }

  return (gpsMsToday() +
          static_cast<uint32_t>(LOG_TIME_OFFSET_SECONDS) * 1000UL) %
         MS_PER_DAY;
}

bool gpsDateTimeIsSane() {
  return gps.date.isValid() && gps.time.isValid() &&
         gps.date.year() >= 2024 &&
         gps.date.month() >= 1 && gps.date.month() <= 12 &&
         gps.date.day() >= 1 && gps.date.day() <= 31;
}

bool gpsLocationIsUsable() {
  return gps.location.isValid() && gps.location.age() <= FIX_MAX_AGE_MS &&
         (gps.location.lat() != 0.0 || gps.location.lng() != 0.0);
}

bool recordingPrerequisitesReady() {
  return sdReady && gpsDateTimeIsSane() && gpsLocationIsUsable();
}

float currentGnssSpeedMps() {
  if (!gps.speed.isValid() || gps.speed.age() > FIX_MAX_AGE_MS) {
    return 0.0f;
  }

  return gps.speed.mps();
}

bool vescDataIsRecent(uint32_t now) {
  return now - lastVescDataMs <= VESC_MAX_AGE_MS;
}

uint32_t logMsToday(uint32_t now) {
  if (!logReady) {
    return gpsLocalMsToday();
  }

  return (logStartMsToday + (now - logStartMillis)) % MS_PER_DAY;
}

void sendUbx(uint8_t messageClass, uint8_t messageId, const uint8_t *payload,
             uint16_t payloadLen) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  auto updateChecksum = [&ckA, &ckB](uint8_t value) {
    ckA = ckA + value;
    ckB = ckB + ckA;
  };

  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);

  gpsSerial.write(messageClass);
  updateChecksum(messageClass);

  gpsSerial.write(messageId);
  updateChecksum(messageId);

  const uint8_t lenLow = payloadLen & 0xFF;
  const uint8_t lenHigh = payloadLen >> 8;
  gpsSerial.write(lenLow);
  updateChecksum(lenLow);
  gpsSerial.write(lenHigh);
  updateChecksum(lenHigh);

  for (uint16_t i = 0; i < payloadLen; i++) {
    gpsSerial.write(payload[i]);
    updateChecksum(payload[i]);
  }

  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
}

void configureGnss5Hz() {
  // UBX-CFG-RATE: measRate=200 ms, navRate=1, timeRef=UTC.
  const uint8_t payload[] = {
      0xC8, 0x00,
      0x01, 0x00,
      0x00, 0x00,
  };

  sendUbx(0x06, 0x08, payload, sizeof(payload));
  Serial.println(F("Requested GNSS update rate: 5 Hz"));
}

void printCsvValue(File &file, float value, uint8_t decimals) {
  file.print(value, decimals);
  file.print(';');
}

void printCsvValue(File &file, int32_t value) {
  file.print(value);
  file.print(';');
}

void printCsvValue(File &file, int value) {
  file.print(value);
  file.print(';');
}

void printCsvValue(File &file, uint32_t value) {
  file.print(value);
  file.print(';');
}

void printCsvValue(File &file, uint8_t value) {
  file.print(value);
  file.print(';');
}

void printCsvValue(String &row, float value, uint8_t decimals) {
  row += String(value, static_cast<unsigned int>(decimals));
  row += ';';
}

void printCsvValue(String &row, int32_t value) {
  row += String(value);
  row += ';';
}

void printCsvValue(String &row, int value) {
  row += String(value);
  row += ';';
}

void printCsvValue(String &row, uint32_t value) {
  row += String(value);
  row += ';';
}

void printCsvValue(String &row, uint8_t value) {
  row += String(value);
  row += ';';
}

void appendToRow(char *row, size_t rowSize, size_t &pos, const char *format, ...) {
  if (pos >= rowSize) {
    return;
  }

  va_list args;
  va_start(args, format);
  int written = vsnprintf(row + pos, rowSize - pos, format, args);
  va_end(args);

  if (written > 0) {
    pos += static_cast<size_t>(written);
    if (pos >= rowSize) {
      pos = rowSize - 1;
    }
  }
}

uint8_t countSemicolons(const char *row) {
  uint8_t count = 0;
  while (*row != '\0') {
    if (*row == ';') {
      count++;
    }
    row++;
  }
  return count;
}

bool openLogFile() {
  if (logReady) {
    return true;
  }

  if (!recordingPrerequisitesReady()) {
    return false;
  }

  uint16_t localYear;
  uint8_t localMonth;
  uint8_t localDay;
  uint8_t localHour;
  uint8_t localMinute;
  uint8_t localSecond;
  localGpsDateTime(localYear, localMonth, localDay,
                   localHour, localMinute, localSecond);

  snprintf(logFileName, sizeof(logFileName), "/%04u-%02u-%02u_%02u-%02u-%02u.csv",
           static_cast<unsigned int>(localYear),
           static_cast<unsigned int>(localMonth),
           static_cast<unsigned int>(localDay),
           static_cast<unsigned int>(localHour),
           static_cast<unsigned int>(localMinute),
           static_cast<unsigned int>(localSecond));

  const bool fileAlreadyExists = SD.exists(logFileName);
  logFile = SD.open(logFileName, FILE_APPEND);
  if (!logFile) {
    Serial.println(F("Could not create VESC CSV log file."));
    return false;
  }

  if (!fileAlreadyExists) {
    logFile.println(VESC_CSV_HEADER);
    logFile.flush();
  }

  logReady = true;
  logStartMillis = millis();
  logStartMsToday = gpsLocalMsToday();
  lastLogFlushMs = millis();
  lastLogCheckpointMs = millis();
  loggedRows = 0;
  lastKnownLogSize = logFile.size();

  Serial.print(F("VESC CSV log: "));
  Serial.println(logFileName);

  return true;
}

bool startRecording(const char *reason) {
  if (loggerState == STATE_RECORDING) {
    return true;
  }

  if (!openLogFile()) {
    return false;
  }

  loggerState = STATE_RECORDING;
  manualStartPending = false;
  speedStartSinceMs = 0;
  erpmStartSinceMs = 0;
  recordingStartedMillis = millis();
  resetStopSpeedWindow(recordingStartedMillis);

  Serial.print(F("Recording started: "));
  Serial.println(reason);
  return true;
}

void stopRecording(const char *reason) {
  if (loggerState == STATE_RECORDING && recordingStartedMillis != 0) {
    lastRecordingDurationMs = millis() - recordingStartedMillis;
  }

  strncpy(lastStopReason, reason, sizeof(lastStopReason) - 1);
  lastStopReason[sizeof(lastStopReason) - 1] = '\0';

  if (logFile) {
    logFile.flush();
    delay(20);
    lastKnownLogSize = logFile.size();
    logFile.close();
  }
  logFile = File();

  logReady = false;
  loggerState = STATE_IDLE;
  recordingStartedMillis = 0;
  resetStopSpeedWindow(millis());
  speedStartSinceMs = 0;
  erpmStartSinceMs = 0;

  Serial.print(F("Recording stopped: "));
  Serial.println(reason);
}

bool checkpointLogFile(uint32_t now) {
  if (!logReady || now - lastLogCheckpointMs < LOG_CHECKPOINT_INTERVAL_MS) {
    return true;
  }

  lastLogCheckpointMs = now;

  if (!logFile) {
    Serial.println(F("SD checkpoint failed: log file is not open."));
    return false;
  }

  logFile.flush();
  delay(10);
  lastKnownLogSize = logFile.size();
  logFile.close();
  logFile = File();

  logFile = SD.open(logFileName, FILE_APPEND);
  if (!logFile) {
    Serial.println(F("SD checkpoint failed: could not reopen log file."));
    sdReady = false;
    return false;
  }

  lastKnownLogSize = logFile.size();
  Serial.print(F("Log checkpoint OK: rows="));
  Serial.print(loggedRows);
  Serial.print(F(", size="));
  Serial.println(lastKnownLogSize);
  return true;
}

void resetStopSpeedWindow(uint32_t now) {
  for (uint8_t i = 0; i < STOP_SPEED_WINDOW_SAMPLES; i++) {
    stopSpeedLowSamples[i] = false;
  }

  lastStopSpeedSampleMs = now;
  stopSpeedSampleIndex = 0;
  stopSpeedSampleCount = 0;
  stopSpeedLowSampleCount = 0;
}

bool updateStopSpeedWindow(uint32_t now, float speedMps) {
  if (now - lastStopSpeedSampleMs < STOP_SPEED_SAMPLE_INTERVAL_MS) {
    return false;
  }

  lastStopSpeedSampleMs = now;
  const bool lowSpeed = speedMps < STOP_SPEED_MPS;

  if (stopSpeedSampleCount >= STOP_SPEED_WINDOW_SAMPLES &&
      stopSpeedLowSamples[stopSpeedSampleIndex]) {
    stopSpeedLowSampleCount--;
  }

  stopSpeedLowSamples[stopSpeedSampleIndex] = lowSpeed;
  if (lowSpeed) {
    stopSpeedLowSampleCount++;
  }

  stopSpeedSampleIndex =
      (stopSpeedSampleIndex + 1) % STOP_SPEED_WINDOW_SAMPLES;
  if (stopSpeedSampleCount < STOP_SPEED_WINDOW_SAMPLES) {
    stopSpeedSampleCount++;
  }

  return stopSpeedSampleCount >= STOP_SPEED_WINDOW_SAMPLES &&
         stopSpeedLowSampleCount >= STOP_SPEED_LOW_SAMPLES_REQUIRED;
}

void configureGnss5HzIfReady() {
  if (gnss5HzConfigured || !gpsDateTimeIsSane()) {
    return;
  }

  configureGnss5Hz();
  gnss5HzConfigured = true;
}

void updateRecordingState(uint32_t now) {
  const float speedMps = currentGnssSpeedMps();
  const bool vescRecent = vescDataIsRecent(now);
  const float motorCurrentAbs = fabs(vescMotorCurrent);
  const float inputCurrentAbs = fabs(vescInputCurrent);
  const float erpmAbs = fabs(vescRpm);

  if (loggerState == STATE_IDLE) {
    if (manualStartPending) {
      if (recordingPrerequisitesReady()) {
        startRecording("manual serial command");
      }
      return;
    }

    if (speedMps > START_SPEED_MPS) {
      if (speedStartSinceMs == 0) {
        speedStartSinceMs = now;
      } else if (now - speedStartSinceMs >= START_CONFIRM_MS) {
        startRecording("GNSS speed");
        return;
      }
    } else {
      speedStartSinceMs = 0;
    }

    if (vescRecent && erpmAbs > START_ERPM_ABS) {
      if (erpmStartSinceMs == 0) {
        erpmStartSinceMs = now;
      } else if (now - erpmStartSinceMs >= START_CONFIRM_MS) {
        startRecording("ERPM");
        return;
      }
    } else {
      erpmStartSinceMs = 0;
    }

    return;
  }

  if (!AUTO_STOP_ENABLED) {
    resetStopSpeedWindow(now);
    return;
  }

  const bool powerIdleForStop =
      !vescRecent ||
      (erpmAbs < STOP_ERPM_ABS && inputCurrentAbs < STOP_INPUT_CURRENT_A);

  if (!powerIdleForStop) {
    resetStopSpeedWindow(now);
    return;
  }

  if (updateStopSpeedWindow(now, speedMps)) {
    stopRecording("auto idle timeout");
  }
}

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  const char command = Serial.read();
  if (command == 's' || command == 'S') {
    manualStartPending = true;
    if (recordingPrerequisitesReady()) {
      startRecording("manual serial command");
    } else {
      Serial.println(F("Manual start pending: waiting for SD and valid GNSS fix/time."));
    }
  } else if (command == 'x' || command == 'X') {
    manualStartPending = false;
    if (loggerState == STATE_RECORDING || logReady) {
      stopRecording("manual serial command");
    } else {
      Serial.println(F("Not recording."));
    }
  }
}

void writeVescCsvRow(uint32_t now) {
  if (!logReady || now - lastLogMs < LOG_INTERVAL_MS) {
    return;
  }

  lastLogMs = now;

  if (!logFile) {
    Serial.println(F("VESC CSV log file is not open."));
    stopRecording("log file error");
    return;
  }

  const uint32_t msToday = logMsToday(now);
  const uint32_t gnssMsToday = gpsLocalMsToday();
  const bool vescRecent = now - lastVescDataMs <= VESC_MAX_AGE_MS;
  const bool gnssUsable = gpsLocationIsUsable();

  char row[768];
  size_t rowPos = 0;
  row[0] = '\0';

  appendToRow(row, sizeof(row), rowPos,
              "%lu;%.1f;%.1f;0;0;0;%.1f;%.2f;%.2f;0;0;%.0f;%.4f;%.4f;%.4f;%.4f;%.4f;%ld;%ld;0;%u;0;0;0;%lu;%.4f;%.4f;%.4f;%.4f;0.000;0.00;%.2f;%.2f;0.000;0.000;0.000;1;%lu;0;0;0;0;0;0;0;0;0;%lu;%.8f;%.8f;%.8f;%.8f;0.00000000;%.8f;%.8f;",
              static_cast<unsigned long>(msToday),
              vescRecent ? vescVoltage : 0.0f,
              vescRecent ? vescTempMos : 0.0f,
              vescRecent ? vescTempMotor : 0.0f,
              vescRecent ? vescMotorCurrent : 0.0f,
              vescRecent ? vescInputCurrent : 0.0f,
              vescRecent ? vescRpm : 0.0f,
              vescRecent ? vescDuty : 0.0f,
              vescRecent ? vescAmpHours : 0.0f,
              vescRecent ? vescAmpHoursCharged : 0.0f,
              vescRecent ? vescWattHours : 0.0f,
              vescRecent ? vescWattHoursCharged : 0.0f,
              static_cast<long>(vescRecent ? vescTachometer : 0),
              static_cast<long>(vescRecent ? vescTachometerAbs : 0),
              static_cast<unsigned int>(vescRecent ? vescFaultCode : 0),
              static_cast<unsigned long>(msToday),
              vescRecent ? vescAmpHours : 0.0f,
              vescRecent ? vescAmpHoursCharged : 0.0f,
              vescRecent ? vescWattHours : 0.0f,
              vescRecent ? vescWattHoursCharged : 0.0f,
              vescRecent ? vescInputCurrent : 0.0f,
              vescRecent ? vescMotorCurrent : 0.0f,
              static_cast<unsigned long>(msToday),
              static_cast<unsigned long>(gnssMsToday),
              gnssUsable ? gps.location.lat() : 0.0,
              gnssUsable ? gps.location.lng() : 0.0,
              gnssUsable && gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
              gnssUsable && gps.speed.isValid() ? gps.speed.mps() : 0.0,
              gnssUsable ? DEFAULT_GNSS_HACC_M : 0.0f,
              gnssUsable ? DEFAULT_GNSS_VACC_M : 0.0f);

  if (countSemicolons(row) != VESC_CSV_SEMICOLONS) {
    skippedBadRows++;
    Serial.println(F("Skipped malformed CSV row."));
    return;
  }

  const size_t expectedWrite = strlen(row) + 2;
  const size_t bytesWritten = logFile.println(row);
  if (bytesWritten != expectedWrite) {
    Serial.println(F("SD short write; stopping recording."));
    stopRecording("SD short write");
    return;
  }

  if (LOG_FLUSH_EVERY_ROW) {
    lastLogFlushMs = now;
    logFile.flush();
  }

  if (!LOG_FLUSH_EVERY_ROW && now - lastLogFlushMs >= LOG_FLUSH_INTERVAL_MS) {
    lastLogFlushMs = now;
    logFile.flush();
  }

  loggedRows++;
  lastKnownLogSize = logFile.size();

  if (!checkpointLogFile(now)) {
    stopRecording("SD checkpoint failed");
  }
}

void updateFixLed(uint32_t now) {
  const bool hasFix =
      gps.location.isValid() && gps.location.age() <= FIX_MAX_AGE_MS;

  if (loggerState == STATE_RECORDING) {
    const bool pulseOff =
        (now % LED_RECORD_PULSE_INTERVAL_MS) < LED_RECORD_PULSE_OFF_MS;
    ledState = !pulseOff;
    digitalWrite(LED_PIN, pulseOff ? LED_OFF : LED_ON);
    return;
  }

  if (hasFix) {
    ledState = true;
    digitalWrite(LED_PIN, LED_ON);
    return;
  }

  if (now - lastLedToggleMs >= LED_GNSS_WAIT_BLINK_INTERVAL_MS) {
    lastLedToggleMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
  }
}

String htmlEscape(String value) {
  value.replace("&", "&amp;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  value.replace("\"", "&quot;");
  return value;
}

bool safeSdPath(const String &path) {
  return path.startsWith("/") && path.indexOf("..") < 0;
}

String formatBytes(uint32_t bytes) {
  if (bytes >= 1024UL * 1024UL) {
    return String(bytes / (1024.0f * 1024.0f), 2) + " MB";
  }

  if (bytes >= 1024UL) {
    return String(bytes / 1024.0f, 1) + " KB";
  }

  return String(bytes) + " B";
}

String formatDuration(uint32_t durationMs) {
  const uint32_t totalSeconds = durationMs / 1000UL;
  if (totalSeconds < 60) {
    return String(totalSeconds) + " s";
  }

  const uint32_t minutes = totalSeconds / 60UL;
  const uint32_t seconds = totalSeconds % 60UL;
  String value = String(minutes) + "m ";
  if (seconds < 10) {
    value += "0";
  }
  value += String(seconds) + "s";
  return value;
}

struct WebFileEntry {
  String path;
  uint32_t size;
};

void appendFileListHtml(String &html) {
  html += F("<section class=\"file-panel\"><div class=\"section-title\">Files</div>");

  if (loggerState == STATE_RECORDING) {
    html += F("<div class=\"empty\">File list locked while recording.</div></section>");
    return;
  }

  if (!ensureSdReady(false)) {
    html += F("<div class=\"empty\">SD card not ready.</div></section>");
    return;
  }

  File root = SD.open("/");
  if (!root) {
    html += F("<div class=\"empty\">Could not open SD root.</div></section>");
    return;
  }

  WebFileEntry files[MAX_WEB_FILES];
  size_t fileCount = 0;
  bool truncated = false;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      break;
    }

    if (!entry.isDirectory()) {
      if (fileCount < MAX_WEB_FILES) {
        files[fileCount].path = String("/") + entry.name();
        files[fileCount].size = entry.size();
        fileCount++;
      } else {
        truncated = true;
      }
    }

    entry.close();
  }
  root.close();

  for (size_t i = 0; i < fileCount; i++) {
    for (size_t j = i + 1; j < fileCount; j++) {
      if (files[j].path < files[i].path) {
        WebFileEntry temp = files[i];
        files[i] = files[j];
        files[j] = temp;
      }
    }
  }

  html += F("<div class=\"files\">");
  for (size_t i = 0; i < fileCount; i++) {
    const String &path = files[i].path;
    html += F("<div class=\"file-row\"><div><div class=\"file-name\">");
    html += htmlEscape(path.substring(1));
    html += F("</div><div class=\"file-meta\">");
    html += formatBytes(files[i].size);
    html += F("</div></div><div class=\"file-actions\">");
    html += F("<a class=\"download\" onclick=\"noteDownload(this)\" data-size=\"");
    html += String(files[i].size);
    html += F("\" href=\"/file?name=");
    html += htmlEscape(path);
    html += F("\">Download</a>");
    html += F("<a class=\"delete\" href=\"/delete?name=");
    html += htmlEscape(path);
    html += F("\">Delete</a></div></div>");
  }
  html += F("</div>");

  if (fileCount == 0) {
    html += F("<div class=\"empty\">No files on SD card.</div>");
  } else if (truncated) {
    html += F("<div class=\"empty\">Showing first 40 files by name.</div>");
  }

  html += F("</section>");
}

void handleRoot() {
  String html;
  html.reserve(9000);
  const bool recording = loggerState == STATE_RECORDING;
  const bool gnssReady = gpsLocationIsUsable();
  const float speedMps = currentGnssSpeedMps();
  const uint32_t activeDurationMs =
      recording && recordingStartedMillis != 0 ? millis() - recordingStartedMillis : 0;

  html += F("<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  html += F("<title>eFoil Logger</title><style>");
  html += F(":root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#101418;color:#eef3f6;font-family:Arial,sans-serif;line-height:1.2}");
  html += F(".wrap{max-width:860px;margin:0 auto;padding:8px}.top{display:flex;justify-content:space-between;gap:8px;align-items:center;margin-bottom:6px;background:#D22EA3;border-radius:8px;padding:7px 8px}");
  html += F("h1{font-size:19px;margin:0;color:white}.sub{color:#ffe5f7;font-size:11px;margin-top:1px}.pill{padding:5px 8px;border-radius:6px;font-weight:700;font-size:11px;letter-spacing:.03em}");
  html += F(".rec{background:#4DE838;color:#071015}.idle{background:#24303a;color:#b9cad4}.pending{background:#b68018;color:white}");
  html += F(".panel{background:#9D9D9D;border:1px solid #828282;border-radius:8px;padding:5px;margin:6px 0}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px}");
  html += F(".metric{background:#101820;border:1px solid #2a3741;border-radius:7px;padding:6px;min-width:0}.label{color:#94a6b1;font-size:9px;text-transform:uppercase;letter-spacing:.04em;white-space:nowrap}.value{font-size:15px;font-weight:700;margin-top:2px;overflow-wrap:anywhere}");
  html += F(".wide{grid-column:span 2}.actions{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:5px;margin:6px 0}.button{display:block;text-align:center;border:0;border-radius:6px;padding:9px 5px;text-decoration:none;font-weight:700;font-size:13px;color:#071015;white-space:nowrap}.start{background:#4DE838}.stop{background:#FF0000}.refresh{background:#0DC0FF}");
  html += F(".position{background:#101820;border:1px solid #2a3741;border-radius:7px;padding:6px;margin:6px 0;color:#c6d5dd;font-size:12px;overflow-wrap:anywhere}.file-panel{background:#9D9D9D;border-radius:8px;padding:6px;margin-top:6px}.section-title{font-size:14px;font-weight:700;margin:0 0 6px;color:#071015}.files{display:flex;flex-direction:column;gap:6px;background:#9D9D9D;border-radius:8px;padding:0}.file-row{display:flex;justify-content:space-between;gap:6px;align-items:center;background:#121a21;border:1px solid #2a3741;border-radius:8px;padding:7px}");
  html += F(".file-row>div:first-child{min-width:0;flex:1}.file-name{font-weight:700;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:13px}.file-meta,.muted{color:#99abb5;font-size:12px}.empty{color:#071015;font-size:12px}.file-actions{display:flex;gap:5px;flex-wrap:nowrap;justify-content:flex-end;flex-shrink:0}.download,.delete{border-radius:6px;padding:7px 8px;text-decoration:none;font-weight:700;font-size:12px;white-space:nowrap}.download{background:#D22EA3;color:white;border:1px solid #f2a7dc}.delete{background:#e8edf0;color:#071015;border:1px solid #9fa9af}");
  html += F(".progress{display:none;position:fixed;left:8px;right:8px;bottom:8px;background:#9D9D9D;border:1px solid #828282;border-radius:8px;padding:8px;z-index:5;color:#071015}.bar{height:10px;background:#101820;border-radius:6px;overflow:hidden;margin-top:6px}.fill{height:100%;width:0;background:#D22EA3}.progress.show{display:block}");
  html += F("@media(min-width:520px){.grid{grid-template-columns:repeat(5,minmax(0,1fr))}.wide{grid-column:span 2}.value{font-size:17px}}");
  html += F("</style></head><body><main class=\"wrap\"><div class=\"top\"><div><h1>eFoil Logger</h1></div><div class=\"pill ");
  html += recording ? F("rec\">RECORDING") : (manualStartPending ? F("pending\">PENDING") : F("idle\">IDLE"));
  html += F("</div></div>");

  html += F("<div class=\"position\">");
  html += F("GPS position: ");
  if (gps.location.isValid()) {
    html += String(gps.location.lat(), 7);
    html += F(" / ");
    html += String(gps.location.lng(), 7);
  } else {
    html += F("Waiting for coordinates");
  }
  if (manualStartPending) {
    html += F("<br>Manual start queued until SD and GNSS are ready.");
  }
  html += F("</div>");

  html += F("<section class=\"panel\"><div class=\"grid\">");
  html += F("<div class=\"metric\"><div class=\"label\">GNSS</div><div class=\"value\">");
  if (gnssReady) {
    html += F("Fix");
    if (gps.satellites.isValid()) {
      html += F(" · ");
      html += String(gps.satellites.value());
      html += F(" sats");
    }
  } else {
    html += F("Waiting");
  }
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">SD</div><div class=\"value\">");
  html += sdReady ? F("OK") : F("No");
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Speed</div><div class=\"value\">");
  html += String(speedMps, 2);
  html += F(" m/s</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">ERPM</div><div class=\"value\">");
  html += String(vescRpm, 0);
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Motor</div><div class=\"value\">");
  html += String(vescMotorCurrent, 1);
  html += F(" A</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Input</div><div class=\"value\">");
  html += String(vescInputCurrent, 1);
  html += F(" A</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Voltage</div><div class=\"value\">");
  html += String(vescVoltage, 1);
  html += F(" V</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Log</div><div class=\"value\">");
  html += logReady ? htmlEscape(String(logFileName).substring(1)) : F("None");
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Duration</div><div class=\"value\">");
  html += formatDuration(recording ? activeDurationMs : lastRecordingDurationMs);
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Rows Written</div><div class=\"value\">");
  html += String(loggedRows);
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">File Size</div><div class=\"value\">");
  html += String(lastKnownLogSize / 1024);
  html += F(" KB");
  html += F("</div></div>");
  html += F("<div class=\"metric\"><div class=\"label\">Skipped Rows</div><div class=\"value\">");
  html += String(skippedBadRows);
  html += F("</div></div>");
  html += F("<div class=\"metric wide\"><div class=\"label\">Last Stop</div><div class=\"value\">");
  html += htmlEscape(String(lastStopReason));
  html += F("</div></div></div></section>");

  html += F("<div class=\"actions\"><a class=\"button start\" href=\"/start\">Start</a>");
  html += F("<a class=\"button stop\" href=\"/stop\">Stop</a>");
  html += F("<a class=\"button refresh\" href=\"/remount-sd\">Renew SD</a>");
  html += F("<a class=\"button refresh\" href=\"/\">Refresh</a></div>");

  appendFileListHtml(html);
  html += F("<div id=\"dlbox\" class=\"progress\"><div id=\"dltext\">Waiting for phone download...</div><div class=\"bar\"><div id=\"dlfill\" class=\"fill\"></div></div></div>");
  html += F("<script>");
  html += F("let rt,dp=null;function sr(){rt=setTimeout(()=>location.reload(),");
  html += String(static_cast<uint16_t>(WEB_REFRESH_SECONDS) * 1000U);
  html += F(")}window.addEventListener('load',sr);function fname(a){return decodeURIComponent((a.href.split('name=')[1]||'log.csv').split('&')[0]).replace(/^\\//,'')}");
  html += F("function noteDownload(a){clearTimeout(rt);dp={name:fname(a),size:parseInt(a.dataset.size||'0'),t:Date.now(),armed:true};sessionStorage.setItem('dlp',JSON.stringify(dp));setTimeout(showEst,1500)}");
  html += F("function showEst(){let s=sessionStorage.getItem('dlp');if(!s)return;dp=JSON.parse(s);if(!dp.armed)return;dp.armed=false;sessionStorage.setItem('dlp',JSON.stringify(dp));let box=document.getElementById('dlbox'),txt=document.getElementById('dltext'),fill=document.getElementById('dlfill');box.classList.add('show');let dur=Math.max(4000,Math.min(60000,dp.size/216));let start=Date.now();let tm=setInterval(()=>{let p=Math.min(98,Math.round((Date.now()-start)*100/dur));fill.style.width=p+'%';txt.textContent='Estimated download '+dp.name+' '+p+'%';if(p>=98){clearInterval(tm);txt.textContent='Waiting for phone download complete sound...';setTimeout(()=>{box.classList.remove('show');sessionStorage.removeItem('dlp');sr()},5000)}},500)}");
  html += F("window.addEventListener('focus',()=>{setTimeout(showEst,300)});window.addEventListener('pageshow',()=>{setTimeout(showEst,300)});");
  html += F("</script></main></body></html>");
  server->send(200, "text/html", html);
}

void handleStart() {
  manualStartPending = true;
  startRecording("manual web command");
  server->sendHeader("Location", "/");
  server->send(303, "text/plain", "");
}

void handleStop() {
  manualStartPending = false;
  if (loggerState == STATE_RECORDING || logReady) {
    stopRecording("manual web command");
  }
  server->send(200, "text/html",
               "<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
               "<meta http-equiv=\"refresh\" content=\"2;url=/\"><title>Stopped</title></head>"
               "<body style=\"font-family:Arial,sans-serif;background:#101418;color:#eef3f6;padding:24px\">"
               "<h1>Recording stopped</h1><p>Returning to logger...</p></body></html>");
}

void handleRemountSd() {
  if (loggerState == STATE_RECORDING) {
    server->send(409, "text/plain", "Stop recording before remounting SD.");
    return;
  }

  remountSd(true);
  server->sendHeader("Location", "/");
  server->send(303, "text/plain", "");
}

void handleFileDownload() {
  if (loggerState == STATE_RECORDING) {
    server->send(409, "text/plain", "Stop recording before downloading files.");
    return;
  }

  if (!ensureSdReady(false)) {
    server->send(503, "text/plain", "SD card not ready.");
    return;
  }

  if (!server->hasArg("name")) {
    server->send(400, "text/plain", "Missing file name.");
    return;
  }

  String path = server->arg("name");
  if (!safeSdPath(path) || !SD.exists(path)) {
    server->send(404, "text/plain", "File not found.");
    return;
  }

  File file = SD.open(path, FILE_READ);
  if (!file) {
    server->send(500, "text/plain", "Could not open file.");
    return;
  }

  String downloadName = path.substring(1);
  server->sendHeader("Content-Disposition", "attachment; filename=\"" + downloadName + "\"");
  server->sendHeader("Content-Length", String(file.size()));
  server->streamFile(file, "text/csv");
  file.close();
}

void handleDeleteConfirm() {
  if (loggerState == STATE_RECORDING) {
    server->send(409, "text/plain", "Stop recording before deleting files.");
    return;
  }

  if (!ensureSdReady(false)) {
    server->send(503, "text/plain", "SD card not ready.");
    return;
  }

  if (!server->hasArg("name")) {
    server->send(400, "text/plain", "Missing file name.");
    return;
  }

  String path = server->arg("name");
  if (!safeSdPath(path) || !SD.exists(path)) {
    server->send(404, "text/plain", "File not found.");
    return;
  }

  String html;
  html.reserve(1800);
  html += F("<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  html += F("<title>Delete Log</title><style>");
  html += F("*{box-sizing:border-box}body{margin:0;background:#101418;color:#eef3f6;font-family:Arial,sans-serif}.wrap{max-width:620px;margin:0 auto;padding:8px}.top{background:#9D9D9D;border:1px solid #828282;border-radius:8px;padding:7px 8px;margin-bottom:6px}h1{font-size:19px;margin:0;color:#071015}.panel{background:#9D9D9D;border:1px solid #828282;border-radius:8px;padding:6px}.box{background:#101820;border:1px solid #2a3741;border-radius:7px;padding:10px}.name{font-weight:700;overflow-wrap:anywhere}.actions{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:5px;margin-top:10px}a{display:block;text-align:center;border-radius:6px;padding:10px 8px;text-decoration:none;font-weight:700;color:#071015}.yes{background:#FF0000;color:white}.no{background:#0DC0FF;color:#071015}");
  html += F("</style></head><body><main class=\"wrap\"><div class=\"top\"><h1>Delete file?</h1></div><section class=\"panel\"><div class=\"box\"><p class=\"name\">");
  html += htmlEscape(path.substring(1));
  html += F("</p><div class=\"actions\"><a class=\"yes\" href=\"/delete-confirm?name=");
  html += htmlEscape(path);
  html += F("\">Delete</a><a class=\"no\" href=\"/\">Cancel</a></div></div></section></main></body></html>");
  server->send(200, "text/html", html);
}

void handleDeleteFile() {
  if (loggerState == STATE_RECORDING) {
    server->send(409, "text/plain", "Stop recording before deleting files.");
    return;
  }

  if (!ensureSdReady(false)) {
    server->send(503, "text/plain", "SD card not ready.");
    return;
  }

  if (!server->hasArg("name")) {
    server->send(400, "text/plain", "Missing file name.");
    return;
  }

  String path = server->arg("name");
  if (!safeSdPath(path) || !SD.exists(path)) {
    server->send(404, "text/plain", "File not found.");
    return;
  }

  if (!SD.remove(path)) {
    server->send(500, "text/plain", "Could not delete file.");
    return;
  }

  server->sendHeader("Location", "/");
  server->send(303, "text/plain", "");
}

void handleCaptivePortalRedirect() {
  server->sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server->send(302, "text/plain", "Redirecting to eFoil Logger");
}

void setupWebServer() {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  IPAddress apIp(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);

  const bool apOk = strlen(WIFI_PASSWORD) >= 8
                        ? WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 6, false, 4)
                        : WiFi.softAP(WIFI_SSID, nullptr, 6, false, 4);
  delay(200);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  Serial.print(F("WiFi AP: "));
  Serial.println(apOk ? F("OK") : F("FAILED"));
  Serial.print(F("SSID: "));
  Serial.println(WIFI_SSID);
  Serial.print(F("Password: "));
  Serial.println(strlen(WIFI_PASSWORD) >= 8 ? WIFI_PASSWORD : "(open network)");
  Serial.print(F("Web page: http://"));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("AP MAC: "));
  Serial.println(WiFi.softAPmacAddress());
  Serial.print(F("AP channel: "));
  Serial.println(WiFi.channel());

  server = new WebServer(80);
  if (!server) {
    Serial.println(F("ERROR: Could not create web server."));
    return;
  }

  server->on("/", handleRoot);
  server->on("/start", handleStart);
  server->on("/stop", handleStop);
  server->on("/remount-sd", handleRemountSd);
  server->on("/file", handleFileDownload);
  server->on("/delete", handleDeleteConfirm);
  server->on("/delete-confirm", handleDeleteFile);
  server->onNotFound(handleCaptivePortalRedirect);
  server->begin();
}

void handleWeb() {
  if (!server) {
    return;
  }

  dnsServer.processNextRequest();
  server->handleClient();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Serial.begin(115200);
  delay(1000);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  vescSerial.begin(VESC_BAUD, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);

  Serial.println();
  Serial.println(F("ESP32 + ELT0169 GNSS test"));
  Serial.println(F("Serial monitor: 115200 baud"));
  Serial.println(F("GNSS UART: GPIO25 RX, GPIO26 TX, 9600 baud, requested 5 Hz"));
  Serial.println(F("VESC UART: GPIO16 RX, GPIO17 TX, 115200 baud, polled at 10 Hz"));
  Serial.println(F("SD SPI: CS=5, SCK=18, MISO=19, MOSI=23, 1 MHz"));
  Serial.println(F("CSV log rows: 5 Hz, flushed every row"));
  Serial.println(F("Serial commands: S=start, X=stop"));
  Serial.println(F("Status LED GPIO27: fast=GNSS waiting, solid=fix, brief off pulse=recording"));
  Serial.println(F("Move the antenna outdoors with its ceramic face toward the sky."));

  testSdCard();
  setupWebServer();
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  const uint32_t now = millis();
  handleWeb();
  handleSerialCommands();
  readVesc();
  pollVesc(now);
  updateFixLed(now);
  configureGnss5HzIfReady();
  updateRecordingState(now);
  writeVescCsvRow(now);
  handleWeb();

  if (!sdReady && now - lastSdRetryMs >= SD_RETRY_INTERVAL_MS) {
    lastSdRetryMs = now;
    ensureSdReady(true);
  }

  if (SERIAL_LIVE_STATUS && now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    printStatus();

    if (now > 5000 && gps.charsProcessed() == 0) {
      Serial.println(F("WARNING: No GNSS data received. Check power, wiring and baud."));
    }
  }
}
