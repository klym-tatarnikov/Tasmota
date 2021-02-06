/*
  xsns_85_ze80.ino - ZE08 formaldehyde sensor support for Tasmota

  Copyright (C) 2021  Klym Tatarnikov

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ZE08
/*********************************************************************************************\
 * ZE08-CH2O formaldehyde gas sensor module
 * Electrochemical principle
 * Output DAC(0.4-2V) and UART
 * https://www.winsen-sensor.com/sensors/ch2o-gas-sensor/ze08-ch2o.html
 * https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/Formaldehyde%20Detection%20Module/ZE08-CH2O%20V1.0.pdf
 * Hardware Serial will be selected if GPIO3 = [ZE08]
\*********************************************************************************************/

#define XSNS_85             85

#include <TasmotaSerial.h>

#ifndef WARMUP_PERIOD
#define WARMUP_PERIOD 30          // Turn on ZE08 XX-seconds before read in passive mode
#endif

#ifndef MIN_INTERVAL_PERIOD
#define MIN_INTERVAL_PERIOD 60    // minimum interval period in seconds required for passive mode
#endif

TasmotaSerial *Ze08Serial;

struct ZE08 {
  uint16_t time = 0;
  uint8_t type = 1;
  uint8_t valid = 0;
  uint8_t wake_mode = 1;
  uint8_t ready = 1;
} Ze08;

enum Ze08Commands
{
  CMD_ZE08_ACTIVE,
  CMD_ZE08_PASSIVE,
  CMD_ZE08_READ_DATA
};

const uint8_t kZe08Commands[][9] PROGMEM = {
    //  0     1    2    3     4     5     6
    {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47},  // ze08_set_active_mode
    {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46},  // ze08_set_passive_mode
    {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}};  // ze08_passive_mode_read

struct ze08data {
  uint8_t start_byte;
  uint8_t gas_name;
  uint8_t UOM;
  uint8_t no_decimal;
  uint8_t concentration_h;
  uint8_t concentration_l;
  uint8_t full_range_h;
  uint8_t full_range_l;
  uint16_t checksum;
} ze08_data;

/*********************************************************************************************/

size_t Ze08SendCmd(uint8_t command_id)
{
  return Ze08Serial->write(kZe08Commands[command_id], sizeof(kZe08Commands[command_id]));
}

/*********************************************************************************************/

bool Ze08ReadData(void)
{
  if (! Ze08Serial->available()) {
    return false;
  }
  while ((Ze08Serial->peek() != 0xFF) && Ze08Serial->available()) {
    Ze08Serial->read();
  }
  if (Ze08Serial->available() < 9) {
    return false;
  }

  uint8_t buffer[9];
  Ze08Serial->readBytes(buffer, 9);
  uint8_t sum = 0;
  Ze08Serial->flush();  // Make room for another burst

  AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, 9);

  // get checksum ready
  for (uint32_t i = 0; i < 9; i++) {
    sum += buffer[i];
  }
  
  if (sum != 0xFF) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("ZE8: " D_CHECKSUM_FAILURE));
    return false;
  }

  memcpy((void *)&ze08_data, (void *)buffer, 9);
  Ze08.valid = 10;

  return true;
}

/*********************************************************************************************\
 * Command Sensor85
 *
 * Warmup time for sensor is 30 seconds, therfore setting interval time to less than 60
 * seconds doesn't really make sense.
 *
 * 0 - 59   -   Active Mode (continuous sensor readings)
 * 60 .. 65535 -  Passive Mode (read sensor every x seconds)
\*********************************************************************************************/

bool Ze08CommandSensor(void)
{
  if (PinUsed(GPIO_ZE08_TX) && (XdrvMailbox.payload >= 0) && (XdrvMailbox.payload < 32001)) {
    if (XdrvMailbox.payload < MIN_INTERVAL_PERIOD) {
      // Set Active Mode if interval is less than 60 seconds
      Settings.pms_wake_interval = 0;
      Ze08.wake_mode = 1;
      Ze08.ready = 1;
      Ze08SendCmd(CMD_ZE08_ACTIVE);
      //Ze08SendCmd(CMD_WAKEUP);
    } else {
      // Set Passive Mode and schedule read once per interval time
      Settings.pms_wake_interval = XdrvMailbox.payload;
      Ze08SendCmd(CMD_ZE08_PASSIVE);
      //Ze08SendCmd(CMD_SLEEP);
      Ze08.wake_mode = 0;
      Ze08.ready = 0;
    }
  }

Response_P(S_JSON_SENSOR_INDEX_NVALUE, XSNS_85, Settings.pms_wake_interval);

  return true;
}

/*********************************************************************************************/

void Ze08Second(void)                 // Every second
{
  if (Settings.pms_wake_interval >= MIN_INTERVAL_PERIOD) {
    // Passive Mode
    Ze08.time++;
    if ((Settings.pms_wake_interval - Ze08.time <= WARMUP_PERIOD) && !Ze08.wake_mode) {
      // wakeup sensor WARMUP_PERIOD before read interval
      Ze08.wake_mode = 1;
  //    Ze08SendCmd(CMD_WAKEUP);
    }
    if (Ze08.time >= Settings.pms_wake_interval) {
      // sensor is awake and warmed up, set up for reading
      Ze08SendCmd(CMD_ZE08_READ_DATA);
      Ze08.ready = 1;
      Ze08.time = 0;
    }
  }

  if (Ze08.ready) {
    if (Ze08ReadData()) {
      Ze08.valid = 10;
      if (Settings.pms_wake_interval >= MIN_INTERVAL_PERIOD) {
        //Ze08SendCmd(CMD_SLEEP);
        Ze08.wake_mode = 0;
        Ze08.ready = 0;
      }
    } else {
      if (Ze08.valid) {
        Ze08.valid--;
        if (Settings.pms_wake_interval >= MIN_INTERVAL_PERIOD) {
          Ze08SendCmd(CMD_ZE08_READ_DATA);
          Ze08.ready = 1;
        }
      }
    }
  }
}

/*********************************************************************************************/

void Ze08Init(void)
{
  Ze08.type = 0;
  if (PinUsed(GPIO_ZE08_RX)) {
    Ze08Serial = new TasmotaSerial(Pin(GPIO_ZE08_RX), (PinUsed(GPIO_ZE08_TX)) ? Pin(GPIO_ZE08_TX) : -1, 1);
    if (Ze08Serial->begin(9600)) {
      if (Ze08Serial->hardwareSerial()) { ClaimSerial(); }

      if (!PinUsed(GPIO_ZE08_TX)) {  // setting interval not supported if TX pin not connected
        Settings.pms_wake_interval = 0;
        Ze08.ready = 1;
      } else {
        if (Settings.pms_wake_interval >= MIN_INTERVAL_PERIOD) {
          // Passive Mode
          Ze08SendCmd(CMD_ZE08_PASSIVE);
          Ze08.wake_mode = 0;
          Ze08.ready = 0;
          Ze08.time = Settings.pms_wake_interval - WARMUP_PERIOD; // Let it wake up in the next second
        }
      }

      Ze08.type = 1;
    }
  }
}

#ifdef USE_WEBSERVER
const char HTTP_ZE08_SNS[] PROGMEM =
  "{s}ZE08 Formaldehyde (HCHO) {m}%d " D_UNIT_PARTS_PER_MILLION "{e}";
#endif  // USE_WEBSERVER

void Ze08Show(bool json)
{
  if (Ze08.valid) {
    uint16_t hcho = ze08_data.concentration_h*256 + ze08_data.concentration_l;
    if (json) {
//      ResponseAppend_P(PSTR(",\"PMS3003\":{\"CF1\":%d,\"CF2.5\":%d,\"CF10\":%d,\"PM1\":%d,\"PM2.5\":%d,\"PM10\":%d}"),
//        pms_data.pm10_standard, pms_data.pm25_standard, pms_data.pm100_standard,
//        pms_data.pm10_env, pms_data.pm25_env, pms_data.pm100_env);
    ResponseAppend_P(PSTR(",\"ZE08\":{\"HCHO\":%d}"),  hcho);
#ifdef USE_DOMOTICZ
      if (0 == TasmotaGlobal.tele_period) {
 //       DomoticzSensor(DZ_COUNT, pms_data.pm10_env);     // PM1
 //       DomoticzSensor(DZ_VOLTAGE, pms_data.pm25_env);   // PM2.5
 //       DomoticzSensor(DZ_CURRENT, pms_data.pm100_env);  // PM10
      }
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    } else {

        WSContentSend_PD(HTTP_ZE08_SNS, (hcho));
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns85(uint8_t function)
{
  bool result = false;

  if (Ze08.type) {
    switch (function) {
      case FUNC_INIT:
        Ze08Init();
        break;
      case FUNC_EVERY_SECOND:
        Ze08Second();
        break;
      case FUNC_COMMAND_SENSOR:
        if (XSNS_85 == XdrvMailbox.index) {
          result = Ze08CommandSensor();
        }
        break;
      case FUNC_JSON_APPEND:
        Ze08Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Ze08Show(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_ZE08
