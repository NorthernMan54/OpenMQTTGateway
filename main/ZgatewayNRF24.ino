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

#  include "RF24.h"

/** nrf24L01 module connections to a ESP32
 * CE_GPIO=25
 * SCK_GPIO=18
 * MISO_GPIO=19
 * CSN_GPIO=5
 * MOSI_GPIO=23 
 * IRQ_GPIO=4
 */

#ifndef CE_GPIO
#define CE_GPIO 25
#endif
#ifndef CSN_GPIO
#define CSN_GPIO 5
#endif

#define DEFAULTCHANNEL 40

RF24 radio(CE_GPIO, CSN_GPIO);

void nrf24setup() {
  Log.notice(F("ZgatewayNRF24 command topic: %s%s" CR), mqtt_topic, subjectMQTTtoNRF24);
  Log.notice(F("ZgatewayNRF24 message topic: %s%s" CR), mqtt_topic, subjectNRF24toMQTT);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(32);
  radio.setChannel(DEFAULTCHANNEL);
  // RF24 doesn't ever fully set this -- only certain bits of it
 // writeRegister(EN_RXADDR, 0x00);
  // RF24 doesn't have a native way to change MAC...
  // 0x00 is "invalid" according to the datasheet, but Travis Goodspeed found it works :)
 // writeRegister(SETUP_AW, 0x00);
 // radio.openReadingPipe(0, promisc_addr0);
 // radio.openReadingPipe(1, promisc_addr1);
  radio.disableCRC();
  radio.startListening();
  radio.stopListening();
  radio.printPrettyDetails();

  Log.trace(F("ZgatewayNRF24 setup done " CR));
}

void nrf24loop() {
}

extern void MQTTtoNRF24(char* topicOri, JsonObject& NRF24data) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoNRF24)) {
    Log.trace(F("MQTTtoNRF24 %s" CR), topicOri);
    float tempMhz = NRF24data["mhz"];
    int minimumRssi = NRF24data["rssi"] | 0;
    int debug = NRF24data["debug"] | -1;
    int status = NRF24data["status"] | -1;
    if (minimumRssi != 0) {
      Log.notice(F("NRF24 minimum RSSI: %d" CR), minimumRssi);
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (debug >= 0 && debug <= 4) {
      Log.notice(F("NRF24 set debug: %d" CR), debug);
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (status >= 0) {
      pub(subjectNRF24toMQTT, NRF24data); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
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
}

extern void disableNRFreceive() {
}

#endif