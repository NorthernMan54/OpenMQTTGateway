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

#  define DATA_RATE       RF24_2MBPS
#  define DEFAULT_CHANNEL 17
#  define ADDRESS_WIDTH   5
#  define ADDRESS1        0xBA151A6F07LL // Logitech mouse
#  define SCANNER_ADDRESS 0xAALL // Scanner address
#  define PAYLOAD_SIZE    32
#  define MIN_CHANNEL     2
#  define MAX_CHANNEL     85
#  define WAIT            100

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

bool sendPayload = false;
bool debug = false;
bool fixedChannel = false;
bool sniffer = false;
bool dynamic = false;

//uint64_t address0 = 0xAALL;
uint64_t address1 = ADDRESS1;
uint64_t writeAddress = 0;
uint8_t addressWidth = 5;
/**
 * nrf24L01 data rate
 * 2 = 250Kbs
 * 0 = 1Mbs
 * 1 = 2Mbs
 */
rf24_datarate_e dataRate = RF24_2MBPS;

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
void convert(const char *s)
{
 int i, j, k;
 buf[0] = 0x0;
 
 for (j = 0, i = 0, k = 0; j < strlen(s); j++)
 {
   
   if (i++ == 0) {
     buf[k++] = '0';
     buf[k++] = 'x';
   }
   
   buf[k++] = s[j];
   
   if (i == 2) {
     if(j != strlen(s) -1)  buf[k++] = ',';
     i = 0;
   }
 }

buf[k] = 0x0;

}

void nrf24setup() {
  Log.notice(F("ZgatewayNRF24 command topic: %s%s" CR), mqtt_topic, subjectMQTTtoNRF24);
  Log.notice(F("ZgatewayNRF24 message topic: %s%s" CR), mqtt_topic, subjectNRF24toMQTT);
  radio.begin();
  radio.setAutoAck(false);
  //radio.write_register(RF_SETUP, 0x09); // Disable PA, 2M rate, LNA enabled
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(dataRate);
#  ifdef PAYLOAD_SIZE
  radio.setPayloadSize(PAYLOAD_SIZE);
#  endif
  radio.setChannel(DEFAULT_CHANNEL);
  radio.setAddressWidth(ADDRESS_WIDTH);
  // RF24 doesn't ever fully set this -- only certain bits of it
  // writeRegister(EN_RXADDR, 0x00);
  // RF24 doesn't have a native way to change MAC...
  // 0x00 is "invalid" according to the datasheet, but Travis Goodspeed found it works :)
  // writeRegister(SETUP_AW, 0x00);
  radio.openReadingPipe(1, address1);
  // radio.openReadingPipe(1, promisc_addr1);
  radio.disableCRC();
  Log.trace(F("ZgatewayNRF24 config CE_GPIO: %d CSN_GPIO: %d" CR), CE_GPIO, CSN_GPIO);
  radio.startListening();
  radio.stopListening();
#  ifdef LOG_LEVEL_TRACE
  radio.printPrettyDetails();
#  endif
  radio.startListening();

  Log.trace(F("ZgatewayNRF24 setup done " CR));
}

/**
 * Enable sniffer mode as per http://travisgoodspeed.blogspot.com/2011/02/promiscuity-is-nrf24l01s-duty.html
 */
void nrf24sniffer() {
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(dataRate);
  radio.setPayloadSize(32);
  radio.setChannel(channel);
  // RF24 doesn't ever fully set this -- only certain bits of it
  nrfWriteRegister(EN_RXADDR, 0x00);
  // RF24 doesn't have a native way to change MAC...
  // 0x00 is "invalid" according to the datasheet, but Travis Goodspeed found it works :)
  nrfWriteRegister(SETUP_AW, 0x00);
  radio.openReadingPipe(1, 0xAALL);
  //  radio.openReadingPipe(1, promisc_addr1);
  radio.disableCRC();
  radio.startListening();
}

void nrf24loop() {
  uint8_t pipe; // initialize pipe data
  if (radio.available(&pipe)) {
    int length = 0;
    if (dynamic) {
      length = radio.getDynamicPayloadSize();
    } else {
      length = radio.getPayloadSize();
    }
    radio.read(&buf, length);
    Log.trace(F("nrf24 channel: %d length: %d pipe: %d payload: "), channel, length, pipe);

    DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
    JsonObject& NRF24Data = jsonBuffer2.createObject();
    NRF24Data.set("channel", (int)channel);
    NRF24Data.set("length", (int)length);
    NRF24Data.set("pipe", (int)pipe);

    JsonArray& payload = NRF24Data.createNestedArray("payload");

    for (int j = 0; j < length; j++) {
      if (buf[j] < 16) {
        Serial.print("0");
        Serial.print(buf[j], HEX);
        payload.add("0" + String(buf[j], HEX));
      } else {
        Serial.print(buf[j], HEX);
        payload.add(String(buf[j], HEX));
      }
      Serial.print(" ");
    }
    Serial.println("");
    if (sendPayload) {
      pub(subjectNRF24toMQTT, NRF24Data);
    }

#  ifdef MEMORY_DEBUG
    Log.trace(F("Post nrf24loop: %d" CR), ESP.getFreeHeap());
#  endif
  }

  if (!fixedChannel) {
    if (millis() - startTime > WAIT) {
      channel++;
      if (channel > MAX_CHANNEL) {
        channel = MIN_CHANNEL;
      }
      radio.setChannel(channel);
      startTime = millis();
      // Log.notice(F("."), channel);
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
    if (NRF24data.containsKey("sendPayload")) {
      sendPayload = NRF24data["sendPayload"];
      Log.notice(F("NRF24 sendPayload: %T" CR), sendPayload);
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (NRF24data.containsKey("sniffer")) {
      sniffer = true;
      nrf24sniffer();
      Log.notice(F("NRF24 sniffer enabled" CR));
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (NRF24data.containsKey("writeAddress")) {
      writeAddress = strtoull(NRF24data["writeAddress"], (char**)'\0', 16);
      radio.openWritingPipe(writeAddress);
      Log.notice(F("NRF24 writeAddress %d" CR), address1);
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("address")) {
      uint64_t _address = strtoull(NRF24data["address"], (char**)'\0', 16);
      sniffer = false;
      address1 = _address;
      radio.openReadingPipe(1, address1);
      Log.notice(F("NRF24 address enabled %d" CR), address1);
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("crc")) {
      int _crc = NRF24data["crc"];
      radio.setCRCLength((rf24_crclength_e)_crc);
      Log.notice(F("NRF24 crc %d" CR), radio.getCRCLength());
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("dynamic")) {
      bool _dynamic = NRF24data["dynamic"];
      dynamic = _dynamic;
      if (dynamic) {
        radio.enableDynamicPayloads();
      } else {
        radio.disableDynamicPayloads();
      }
      Log.notice(F("NRF24 dynamic %T" CR), dynamic);
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("payloadSize")) {
      int _payloadSize = NRF24data["payloadSize"];
      radio.setPayloadSize(_payloadSize);
      Log.notice(F("NRF24 payloadSize %d" CR), radio.getPayloadSize());
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("addressWidth")) {
      int _addressWidth = NRF24data["addressWidth"];
      addressWidth = _addressWidth;
      radio.setAddressWidth(addressWidth);
      Log.notice(F("NRF24 addressWidth %d" CR), addressWidth);
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("channel")) {
      int _channel = NRF24data["channel"];
      if (_channel < 0) {
        fixedChannel = false;
        Log.notice(F("NRF24 channel scanner enabled." CR));
      } else {
        fixedChannel = true;
        channel = _channel;
        Log.notice(F("NRF24 channel set to: %d" CR), channel);
      }
      Log.notice(F("NRF24 set debug: %d" CR), debug);
      pub(subjectNRF24toMQTT, NRF24data);
    } else if (NRF24data.containsKey("dataRate")) {
      uint8_t _dataRate = NRF24data["dataRate"];
      if (_dataRate <= 2) {
        dataRate = (rf24_datarate_e)_dataRate;
        Log.notice(F("NRF24 set dataRate: %d" CR), dataRate);
        radio.setDataRate(dataRate);
        pub(subjectNRF24toMQTT, NRF24data);
      } else {
        pub(subjectNRF24toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
        Log.error(F("MQTTtoNRF24 illegal data rate." CR));
      }
    } else if (NRF24data.containsKey("debug")) {
      debug = NRF24data["debug"];
      Log.notice(F("NRF24 set debug: %d" CR), debug);
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (NRF24data.containsKey("status")) {
      radio.printPrettyDetails();
      nrf24Status();
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (NRF24data.containsKey("write")) {
      const char* message = NRF24data.get<const char*>("write");
      // char * message = NRF24data["write"];
      convert(message);
      radio.stopListening();
      radio.write(&buf, strlen(message)/2);
      radio.startListening();
      Log.notice(F("NRF24 write channel: %d, length: %d \"%s\"" CR), channel, strlen(message)/2, NRF24data.get<const char*>("write"));
      pub(subjectNRF24toMQTT, NRF24data);
    } else {
      pub(subjectNRF24toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
      Log.error(F("MQTTtoNRF24 Fail json" CR));
    }
    // enableActiveReceiver();
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