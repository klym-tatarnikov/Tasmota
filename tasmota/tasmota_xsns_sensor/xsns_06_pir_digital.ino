/*
  xsns_06_dht.ino - DHTxx, AM23xx and SI7021 temperature and humidity sensor support for Tasmota

  SPDX-FileCopyrightText: 2022 Theo Arends

  SPDX-License-Identifier: GPL-3.0-only
*/

#ifdef USE_PIR_DIGITAL
/*********************************************************************************************\
 * DHT11, AM2301 (DHT21, DHT22, AM2302, AM2321), SI7021, THS01, MS01 - Temperature and Humidity
 *
 * Reading temperature or humidity takes about 250 milliseconds!
 * Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
 *
 * Changelog
 * 20230215 - v7
 *          - Add user high and low delay in microseconds
 *            DhtDelay1        - Show delays for first sensor
 *            DhtDelay1 1      - Reset to defaults
 *            DhtDelay1 500,40 - Set both delays for sensor 1
 *            DhtDelay4 500,40 - Set both delays for sensor 4
 * 20220706 - v6
 *          - Consolidate Adafruit DHT library
 *          - Fix ESP32 interrupt control to solve intermittent results
 * 20211229 - Change poll time from to 2 to 4 seconds for better results
 * 20211226 - https://github.com/arendst/Tasmota/pull/14173
 * 20210524 - https://github.com/arendst/Tasmota/issues/12180
 * 20200621 - https://github.com/arendst/Tasmota/pull/7468#issuecomment-647067015
 * 20200313 - https://github.com/arendst/Tasmota/issues/7717#issuecomment-585833243
\*********************************************************************************************/

#define XSNS_06          6

#ifndef DHT_MAX_SENSORS
#define DHT_MAX_SENSORS  4
#endif
#define DHT_MAX_RETRY    8

#define BUFFER_LEN 100
uint16_t pirData[BUFFER_LEN];
uint8_t count = 0;
volatile uint16_t movement = 0;
uint16_t movement_threshold = 1000;
uint16_t movement_timeout = 40;
const uint16_t dht_delays_const[4][2] = {
  { 1000, 50 },   // DHT11
  { 2000, 50 },    // DHT22
#ifdef ESP8266
  { 500, 30 },     // SI7021 / THS-01
  { 450, 30 }      // MS01
#else
  { 400, 30 },     // SI7021 / THS-01
  { 400, 30 }      // MS01
#endif
};

uint32_t dht_maxcycles;
uint8_t dht_data[5];
uint8_t dht_sensors = 0;
uint8_t dht_pin;
uint8_t dht_pin_out = 0;                      // Shelly GPIO00 output only
bool dht_active = true;                       // DHT configured
bool dht_dual_mode = false;                   // Single pin mode

struct DHTSTRUCT {
  float    t = NAN;
  float    h = NAN;
  uint16_t delay_lo;
  uint16_t delay_hi;
  uint16_t type;
  uint32_t  raw;
  char     stype[12];
  int8_t   pin;
  uint8_t  lastresult;
} Dht[DHT_MAX_SENSORS];

 

bool DhtRead(uint32_t sensor) {
  dht_pin = Dht[sensor].pin;
   
  

  float temperature = NAN;
  float humidity = NAN;
  uint32_t pir = 0;
  noInterrupts();
  
  pinMode(dht_pin, OUTPUT);
 
  digitalWrite(dht_pin, 1); 
  delayMicroseconds(110);
  for (volatile uint8_t i=0;i<19;i++)
    {
    pinMode(dht_pin, OUTPUT);
    digitalWrite(dht_pin, 0);    
    delayMicroseconds(2);
  
    digitalWrite(dht_pin, 1);    
    delayMicroseconds(2);
  
    pinMode(dht_pin, INPUT);
    delayMicroseconds(6);
    pir = (pir<<1) | digitalRead(dht_pin);
    delayMicroseconds(5);
  }

  interrupts();
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("DHT: Pin%d read %4X"), dht_pin, pir);
  if ((pir&0b1100000000000000001)!=0b1000000000000000000) //wrong packet
  {
    return false;
  }
  Dht[sensor].t = 1;

  
  pir = pir>>1;
  int16_t v = uint16_t(pir&0xFFFF);
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("DHT: Pin%d value %d"), dht_pin, v);
  Dht[sensor].h = v;
  Dht[sensor].raw = pir;
  
  Dht[sensor].lastresult = 0;
  pirData[count]=abs(v);
    count++;
    if (count==BUFFER_LEN)
      count = 0;
  if (abs(v)>movement_threshold)
  {
    if (!movement)
    {
      movement = movement_timeout; 
      MqttPublishSensor();
    } else
    {
      movement = movement_timeout; 
    }
  }
  return true;
}
void CheckThreshold(void)
{
  if (movement)
  {
    movement--;
    if (!movement)
      MqttPublishSensor();
  }
}


/********************************************************************************************/

void DhtDelayDefault(uint32_t sensor) {
  uint32_t index = Dht[sensor].type - GPIO_DHT11;  // GPIO_DHT11, GPIO_DHT22, GPIO_SI7021
  if (index > 2) { index = 3; }                    // GPIO_MS01
  Dht[sensor].delay_lo = dht_delays_const[index][0];
  Dht[sensor].delay_hi = dht_delays_const[index][1];
}

 

void DhtInit(void) {
  if (PinUsed(GPIO_DHT11) ) {
    
    dht_sensors = 1;
    Dht[0].pin = Pin(GPIO_DHT11);
    snprintf_P(Dht[0].stype, sizeof(Dht[0].stype), PSTR("PIR%c%02d"), IndexSeparator(), Dht[0].pin);
    AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: (v7) " D_SENSORS_FOUND " %d, pin:"), dht_sensors, Dht[0].pin);
  } else {
    dht_active = false;
  }}


void DhtShow(bool json) {
  for (uint32_t i = 0; i < dht_sensors; i++) {
      uint32_t sum = 0;
      for (int i=0;i<BUFFER_LEN;i++)  
        sum+=pirData[i];
      sum = sum/BUFFER_LEN;  
      if (json) {
        ResponseAppend_P(PSTR(",\"%s\":{\"" D_JSON_DISTANCE "\":%*d,\"Raw\":%d, \"Movement\":%d}"),
          Dht[i].stype, Settings->flag2.humidity_resolution, sum, Dht[i].raw, movement);
#ifdef USE_WEBSERVER
      } else {
        char parameter[FLOATSZ];
        dtostrfd(sum, Settings->flag2.humidity_resolution, parameter);
        WSContentSend_PD(HTTP_SNS_HUM, Dht[i].stype, parameter);
        
        WSContentSend_PD(HTTP_SNS_ANALOG, Dht[i].stype,i, Dht[0].raw);
        WSContentSend_PD(HTTP_SNS_RANGE, Dht[i].stype, movement);
#endif  // USE_WEBSERVER
      } 
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

const char kDhtCommands[] PROGMEM = "Dht|"  // Prefix
  "Delay";

void (* const DhtCommand[])(void) PROGMEM = {
  &CmndDhtDelay };

void CmndDhtDelay(void) {
  // DhtDelay1        - Show delays for first sensor
  // DhtDelay1 1      - Reset to defaults
  // DhtDelay1 500,40 - Set both delays for sensor 1
  // DhtDelay4 500,40 - Set both delays for sensor 4
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= dht_sensors)) {
    uint32_t sensor = XdrvMailbox.index -1;
    if (XdrvMailbox.data_len > 0) {
      uint32_t parm[2] = { Dht[sensor].delay_lo, Dht[sensor].delay_hi };
      ParseParameters(2, parm);
      if (1 == parm[0]) {
        DhtDelayDefault(sensor);
      } else {
        Dht[sensor].delay_lo = parm[0];
        Dht[sensor].delay_hi = parm[1];
      }
    }
    Response_P(PSTR("{\"%s%d\":[%d,%d]}"), XdrvMailbox.command, XdrvMailbox.index, Dht[sensor].delay_lo, Dht[sensor].delay_hi);
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns06(uint32_t function) {
  bool result = false;

  if (dht_active) {
    switch (function) {
      
      case FUNC_EVERY_50_MSECOND:
        DhtRead(0);
        break;
      case FUNC_EVERY_SECOND:
        CheckThreshold();
        break;
      case FUNC_JSON_APPEND:
        DhtShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        DhtShow(0);
        break;
#endif  // USE_WEBSERVER
      case FUNC_COMMAND:
        result = DecodeCommand(kDhtCommands, DhtCommand);
        break;
      case FUNC_INIT:
        DhtInit();
        break;
    }
  }
  return result;
}

#endif  // USE_PIR_DIGITAL
