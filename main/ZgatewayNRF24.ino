/*  
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR signal  and a MQTT broker 
   Send and receiving command by MQTT
 
  This gateway enables to:
 - receive MQTT data from a topic and send RF 433Mhz signal corresponding to the received MQTT data
 - publish MQTT data to a different topic related to received 433Mhz signal

    Copyright: (c)Florian ROBERT
  
    This file is part of OpenMQTTGateway.
    
    OpenMQTTGateway is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenMQTTGateway is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "User_config.h"

#ifdef ZgatewayNRF24

#  include <SPI.h>

#  include "RF24.h"
#  include "nRF24L01.h"

#  define DATA_RATE       RF24_250KBPS // Connex Dimplex Baseboard
#  define DEFAULT_CHANNEL 40
#  define ADDRESS_WIDTH   5
// #  define ADDRESS1        0xBA151A6F07LL // Logitech mouse
#  define WADDRESS        0x5049382716LL // Connex Dimplex baseboard
#  define ADDRESS1        0x5049382716LL // Connex Dimplex baseboard
#  define SCANNER_ADDRESS 0xAALL // Scanner address
#  define PAYLOAD_SIZE    10
#  define CRCSIZE         2 // default to 16 bit crc / Connex Dimplex baseboard
#  define AUTOACK         true
#  define DEBUG           true

#  define MIN_CHANNEL 2
#  define MAX_CHANNEL 125
#  define WAIT        100

#  define PKT_SIZE 37

#  define JSON_MSG_BUFFER 512

/** nrf24L01 module connections to a ESP32
 * CE_GPIO=25
 * SCK_GPIO=18
 * MISO_GPIO=19
 * CSN_GPIO=5
 * MOSI_GPIO=23 
 * IRQ_GPIO=4
 */

#  ifndef CE_GPIO
#    define CE_GPIO 25
#  endif
#  ifndef CSN_GPIO
#    define CSN_GPIO 5
#  endif

RF24 radio(CE_GPIO, CSN_GPIO);
uint8_t buf[PKT_SIZE];
uint8_t channel = DEFAULT_CHANNEL;
unsigned long startTime = millis();
uint8_t values[MAX_CHANNEL + 1];

bool sendPayload = false;
bool debug = DEBUG;
bool fixedChannel = true;
bool sniffer = false;
bool dynamic = false;

uint64_t address1 = ADDRESS1;
uint64_t writeAddress = WADDRESS;
uint8_t addressWidth = ADDRESS_WIDTH;
bool autoAck = AUTOACK;
/**
 * nrf24L01 data rate
 * 2 = 250Kbs
 * 0 = 1Mbs
 * 1 = 2Mbs
 */
rf24_datarate_e dataRate = DATA_RATE;

uint8_t nrfWriteRegister(uint8_t reg, uint8_t value) {
  uint8_t status;

  digitalWrite(CSN_GPIO, LOW);
  status = SPI.transfer(W_REGISTER | (REGISTER_MASK & reg));
  SPI.transfer(value);
  digitalWrite(CSN_GPIO, HIGH);
  return status;
}

uint8_t nrfWriteRegister(uint8_t reg, const uint8_t* buf, uint8_t len) {
  uint8_t status;

  digitalWrite(CSN_GPIO, LOW);
  status = SPI.transfer(W_REGISTER | (REGISTER_MASK & reg));
  while (len--)
    SPI.transfer(*buf++);
  digitalWrite(CSN_GPIO, HIGH);

  return status;
}

/**
 * convert hexadecimal string to buffer for sending
 */
void convert(const char* s) {
  int i, j, k;
  buf[0] = 0x0;
  for (j = 0, i = 0, k = 0; j < strlen(s); j++) {
    uint8_t letter = hexchartoint(s[j]) * 16;
    letter = letter + hexchartoint(s[j + 1]);
    buf[k] = letter;
    j++;
    k++;
  }

  buf[k] = 0x0;
}

uint8_t hexchartoint(char hex) {
  if (hex >= '0' && hex <= '9')
    return hex - '0';

  if (hex >= 'A' && hex <= 'F')
    return (hex - 'A') + 10;

  if (hex >= 'a' && hex <= 'f')
    return (hex - 'a') + 10;

  return 0;
}

void printBuf(int length) {
  for (int j = 0; j < length; j++) {
    if (buf[j] < 16) {
      if (debug) {
        Serial.print("0");
        Serial.print(buf[j], HEX);
      }

    } else {
      if (debug) {
        Serial.print(buf[j], HEX);
      }
    }
    if (debug) {
      Serial.print(" ");
    }
  }
  if (debug) {
    Serial.println("");
  }
}

void nrf24setup() {
  Log.notice(F("ZgatewayNRF24 command topic: %s%s" CR), mqtt_topic, subjectMQTTtoNRF24);
  Log.notice(F("ZgatewayNRF24 message topic: %s%s" CR), mqtt_topic, subjectNRF24toMQTT);
  radio.begin();
  radio.setAutoAck(false);
  radio.disableCRC();
  //radio.write_register(RF_SETUP, 0x09); // Disable PA, 2M rate, LNA enabled
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(dataRate);
  radio.setPayloadSize(PAYLOAD_SIZE);
  radio.setChannel(channel);
  // writeRegister(EN_RXADDR, 0x00);
  radio.setAddressWidth(addressWidth);
  radio.setCRCLength((rf24_crclength_e)CRCSIZE);
  radio.openReadingPipe(0, address1);
  radio.setAutoAck(0, autoAck);
  radio.openWritingPipe(writeAddress);
  Log.trace(F("ZgatewayNRF24 config CE_GPIO: %d CSN_GPIO: %d" CR), CE_GPIO, CSN_GPIO);
  radio.startListening();
  radio.stopListening();
#  ifdef LOG_LEVEL_TRACE
  radio.printPrettyDetails();
#  endif
  radio.startListening();
  memset(values, 0, sizeof(values));
  Log.trace(F("ZgatewayNRF24 setup done " CR));
}

/**
 * Enable sniffer mode as per http://travisgoodspeed.blogspot.com/2011/02/promiscuity-is-nrf24l01s-duty.html
 */
void nrf24sniffer() {
  radio.begin();
  radio.stopListening();
  radio.setAutoAck(false);
  nrfWriteRegister(RF_SETUP, 0x09);
  radio.setPayloadSize(32);
  radio.setChannel(25);
  // RF24 doesn't ever fully set this -- only certain bits of it
  nrfWriteRegister(EN_RXADDR, 0x00);
  nrfWriteRegister(EN_AA, 0x00); // disable shockburst auto acknowledge
  nrfWriteRegister(SETUP_RETR, 0x00); // disable retransmit
  // RF24 doesn't have a native way to change MAC...
  // 0x00 is "invalid" according to the datasheet, but Travis Goodspeed found it works :)
  nrfWriteRegister(SETUP_AW, 0x00);
  radio.openReadingPipe(0, 0xAALL);

  radio.disableCRC();
  radio.startListening();
}

void nrf24loop() {
  uint8_t pipe; // initialize pipe data
  // if (radio.isChipConnected() && radio.available(&pipe)) {
  if (radio.available(&pipe)) {
    int length = 0;
    ++values[channel];
    if (dynamic) {
      length = radio.getDynamicPayloadSize();
    } else {
      length = radio.getPayloadSize();
    }
    radio.read(&buf, length);
    if (!sniffer) {
      if (debug) {
        Log.trace(F("nrf24 channel: %d length: %d pipe: %d payload: "), channel, length, pipe);
      }
      DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
      JsonObject& NRF24Data = jsonBuffer2.createObject();
      NRF24Data.set("channel", (int)channel);
      NRF24Data.set("length", (int)length);
      NRF24Data.set("pipe", (int)pipe);

      JsonArray& payload = NRF24Data.createNestedArray("payload");

      for (int j = 0; j < length; j++) {
        if (buf[j] < 16) {
          if (debug) {
            Serial.print("0");
            Serial.print(buf[j], HEX);
          }
          payload.add("0" + String(buf[j], HEX));
        } else {
          if (debug) {
            Serial.print(buf[j], HEX);
          }
          payload.add(String(buf[j], HEX));
        }
        if (debug) {
          Serial.print(" ");
        }
      }
      if (debug) {
        Serial.println("");
      }
      if (sendPayload) {
        pub(subjectNRF24toMQTT, NRF24Data);
      }

#  ifdef MEMORY_DEBUG
      Log.trace(F("Post nrf24loop: %d" CR), ESP.getFreeHeap());
#  endif
    }
  }

  if (!fixedChannel) {
    if (millis() - startTime > WAIT) {
      channel++;
      if (channel > MAX_CHANNEL) {
        channel = MIN_CHANNEL;
        if (sniffer) {
          uint8_t busiestChannel = 0;
          uint8_t maxUsage = 0;
          uint8_t i = 0;
          while (i <= MAX_CHANNEL) {
            if (values[i] >= 16) {
              Serial.print("*");
            } else {
              Serial.print(min((uint8_t)0xf, values[i]), HEX);
            }
            if (values[i] > maxUsage) {
              busiestChannel = i;
              maxUsage = values[i];
            }
            ++i;
          }
          // if ((uint8_t)maxUsage > 0) {
          Serial.print(" busy: ");
          Serial.print((uint8_t)busiestChannel);
          Serial.print(" count: ");
          Serial.print((uint8_t)maxUsage);
          Serial.print(" uptime: ");
          Serial.println(millis() / 1000);
          // }
        }
        memset(values, 0, sizeof(values));
      }
      radio.setChannel(channel);
      startTime = millis();
      //      Log.notice(F("."), channel);
    }
  }
}

/**
 * return json device status
 */
void nrf24Status() {
  DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
  JsonObject& NRF24Data = jsonBuffer2.createObject();
  if (fixedChannel) {
    NRF24Data.set("channel", (int)channel);
  } else {
    NRF24Data.set("channel", -1);
  }
  if (sniffer) {
    NRF24Data.set("sniffer", (bool)sniffer);
  } else {
    NRF24Data.set("address", address1);
  }
  if (writeAddress > 0) {
    NRF24Data.set("writeAddress", writeAddress);
  }
  NRF24Data.set("addressWidth", (int)addressWidth);
  NRF24Data.set("payloadSize", radio.getPayloadSize());
  NRF24Data.set("crc", radio.getCRCLength());
  NRF24Data.set("payloadSize", radio.getPayloadSize());
  NRF24Data.set("dynamic", (bool)dynamic);
  NRF24Data.set("autoAck", (bool)autoAck);
  NRF24Data.set("connected", (bool)radio.isChipConnected());
  pub(subjectNRF24toMQTT, NRF24Data);
}

/**
 * nrf24 commands
 * 
 * modes  - scanner - continuiosly loop thru all available channels ( set channel to -1 to enable, set channel to a valid channel to disable )
 *        - sniffer - enable address sniffer mode ( sniffer = true, to disable set an address )
 *        - normal  - fixed channel and address
 * 
 * sendPayload  - Send payload as a mqtt message
 * dataRate     - Set date rate ( 0 - 1 Mbs, 1 - 2Mbs, 2 - 250Kbs )
 * status       - print to stdout the current nrf24l01 status
 * debug        - set debug level
 * channel      - set receive channel, 0 to 125.  -1 enable channel scanner
 * sniffer      - enable sniffer
 * address      - set receive address ( disable sniffer )
 * writeAddress - set write address
 * addressWidth - set address width
 * crc          - enable crc and length 1 or 2 bytes, 0 to disable
 * payloadSize  - set payload size
 * dynamic      - enable dynamic payload length
 * write        - hexidecimal string to be transmitted
 * 
 */

extern void MQTTtoNRF24(char* topicOri, JsonObject& NRF24data) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoNRF24)) {
    Log.trace(F("MQTTtoNRF24 %s" CR), topicOri);
    bool success = false;
    if (NRF24data.containsKey("sendPayload")) {
      sendPayload = NRF24data["sendPayload"];
      Log.notice(F("NRF24 sendPayload: %T" CR), sendPayload);
      success = true;
    };
    if (NRF24data.containsKey("sniffer")) {
      sniffer = true;
      fixedChannel = false;
      nrf24sniffer();
      Log.notice(F("NRF24 sniffer enabled" CR));
      success = true;
    };
    if (NRF24data.containsKey("writeAddress")) {
      writeAddress = strtoull(NRF24data["writeAddress"], (char**)'\0', 16);
      radio.stopListening();
      sniffer = false;
      radio.openWritingPipe(writeAddress);
      radio.startListening();
      Log.notice(F("NRF24 writeAddress %d" CR), address1);
      success = true;
    };
    if (NRF24data.containsKey("ack")) {
      bool _ack = NRF24data["ack"];
      radio.stopListening();
      sniffer = false;
      radio.setAutoAck(1, _ack);
      radio.startListening();
      autoAck = _ack;
      Log.notice(F("NRF24 ack %d" CR), autoAck);
      success = true;
    };
    if (NRF24data.containsKey("address")) {
      uint64_t _address = strtoull(NRF24data["address"], (char**)'\0', 16);
      sniffer = false;
      address1 = _address;
      radio.stopListening();
      radio.openReadingPipe(0, address1);
      radio.startListening();
      Log.notice(F("NRF24 address enabled %d" CR), address1);
      success = true;
    };
    if (NRF24data.containsKey("crc")) {
      int _crc = NRF24data["crc"];
      radio.startListening();
      sniffer = false;
      radio.setCRCLength((rf24_crclength_e)_crc);
      radio.startListening();
      Log.notice(F("NRF24 crc %d" CR), radio.getCRCLength());
      success = true;
    };
    if (NRF24data.containsKey("dynamic")) {
      bool _dynamic = NRF24data["dynamic"];
      dynamic = _dynamic;
      radio.stopListening();
      sniffer = false;
      if (dynamic) {
        radio.enableDynamicPayloads();
      } else {
        radio.disableDynamicPayloads();
      }
      radio.startListening();
      Log.notice(F("NRF24 dynamic %T" CR), dynamic);
      success = true;
    };
    if (NRF24data.containsKey("payloadSize")) {
      int _payloadSize = NRF24data["payloadSize"];
      radio.stopListening();
      sniffer = false;
      radio.setPayloadSize(_payloadSize);
      radio.startListening();
      Log.notice(F("NRF24 payloadSize %d" CR), radio.getPayloadSize());
      success = true;
    };
    if (NRF24data.containsKey("addressWidth")) {
      int _addressWidth = NRF24data["addressWidth"];
      addressWidth = _addressWidth;
      radio.stopListening();
      sniffer = false;
      radio.setAddressWidth(addressWidth);
      radio.startListening();
      Log.notice(F("NRF24 addressWidth %d" CR), addressWidth);
      success = true;
    };
    if (NRF24data.containsKey("channel")) {
      int _channel = NRF24data["channel"];
      sniffer = false;
      if (_channel < 0) {
        fixedChannel = false;
        Log.notice(F("NRF24 channel scanner enabled." CR));
      } else {
        fixedChannel = true;
        channel = _channel;
        radio.setChannel(channel);
        Log.notice(F("NRF24 channel set to: %d" CR), channel);
      }
      Log.notice(F("NRF24 set debug: %d" CR), debug);
      success = true;
    };
    if (NRF24data.containsKey("dataRate")) {
      uint8_t _dataRate = NRF24data["dataRate"];
      sniffer = false;
      if (_dataRate <= 2) {
        dataRate = (rf24_datarate_e)_dataRate;
        Log.notice(F("NRF24 set dataRate: %d" CR), dataRate);
        radio.stopListening();
        radio.setDataRate(dataRate);
        radio.startListening();
        success = true;
      } else {
        pub(subjectNRF24toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
        Log.error(F("MQTTtoNRF24 illegal data rate." CR));
      }
    };
    if (NRF24data.containsKey("debug")) {
      debug = NRF24data["debug"];
      Log.notice(F("NRF24 set debug: %d" CR), debug);
      success = true;
    };
    if (NRF24data.containsKey("status")) {
      radio.printPrettyDetails();
      nrf24Status();
      success = true;
    };
    if (NRF24data.containsKey("write")) {
      const char* write = NRF24data.get<const char*>("write");
      // char * message = NRF24data["write"];
      convert(write);
      radio.stopListening();
      bool writeStatus = radio.write(buf, strlen(write) / 2);
      Log.notice(F("NRF24 write channel: %d, length: %d, sent: %T, \"%s\"" CR), channel, strlen(write) / 2, writeStatus, write);
      printBuf(strlen(write) / 2);
      if (!writeStatus) {
        pub(subjectNRF24toMQTT, "{\"Status\": \"Write Error\"}"); // Fail feedback
        Log.error(F("MQTTtoNRF24 write fail" CR));
      }
      radio.startListening();
      success = true;
    }
    if (!success) {
      pub(subjectNRF24toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
      Log.error(F("MQTTtoNRF24 Fail json" CR));
    } else {
      pub(subjectNRF24toMQTT, NRF24data);
    }
  }
}

extern void enableNRFreceive() {
  Log.trace(F("enableNRF24receive: " CR));
#  ifdef ZgatewayRF
  disableRFReceive();
#  endif
#  ifdef ZgatewayPilight
  disablePilightReceive();
#  endif
#  ifdef ZgatewayRTL_433
  disableRTLreceive();
#  endif
  radio.startListening();
}

extern void disableNRFreceive() {
  radio.stopListening();
}

#endif