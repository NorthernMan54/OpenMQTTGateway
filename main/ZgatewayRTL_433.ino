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

#ifdef ZgatewayRTL_433

#  include <rtl_433_ESP.h>

#  define CC1101_FREQUENCY 433.92
#  define JSON_MSG_BUFFER  512
#  define ONBOARD_LED      2

char messageBuffer[JSON_MSG_BUFFER];

rtl_433_ESP rtl_433(-1); // use -1 to disable transmitter

#  include <ELECHOUSE_CC1101_SRC_DRV.h>

void rtl_433_Callback(char* message) {
  DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
  JsonObject& RFrtl_433_ESPdata = jsonBuffer2.parseObject(message);

  pub(subjectRTL_433toMQTT, RFrtl_433_ESPdata);
#  ifdef MEMORY_DEBUG
    Log.trace(F("Post rtl_433_Callback: %d" CR), ESP.getFreeHeap());
#  endif
}

void rtl_433setup() {
  rtl_433.initReceiver(RF_RECEIVER_GPIO, CC1101_FREQUENCY);
  rtl_433.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
  Log.trace(F("ZgatewayRTL_433 setup done " CR));
}

void rtl_433loop() {
  rtl_433.loop();
}

extern void MQTTtoRTL_433(char* topicOri, JsonObject& RTLdata) {
  if (cmpToMainTopic(topicOri, subjectMQTTtoRTL_433)) {
    Log.trace(F("MQTTtoRTL_433 %s" CR), topicOri);
    float tempMhz = RTLdata["mhz"];
    int minimumRssi = RTLdata["rssi"] | 0;
    int debug = RTLdata["debug"] | 0;
    if (tempMhz != 0 && validFrequency((int)tempMhz)) {
      activeReceiver = RTL; // Enable RTL_433 Gateway
      receiveMhz = tempMhz;
      Log.notice(F("RTL_433 Receive mhz: %F" CR), receiveMhz);
      pub(subjectRTL_433toMQTT, RTLdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (minimumRssi != 0) {      
      Log.notice(F("RTL_433 minimum RSSI: %d" CR), minimumRssi);
      rtl_433.setMinimumRSSI(minimumRssi);
      pub(subjectRTL_433toMQTT, RTLdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else if (debug != 0) {      
      Log.notice(F("RTL_433 gather debug: %d" CR), debug);
      rtl_433.getDebug(debug);
      pub(subjectRTL_433toMQTT, RTLdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else {
      pub(subjectRTL_433toMQTT, "{\"Status\": \"Error\"}"); // Fail feedback
      Log.error(F("MQTTtoRTL_433 Fail json" CR));
    }
    enableActiveReceiver();
  }
}

extern void enableRTLreceive() {
  Log.trace(F("enableRTLreceive: %F" CR), receiveMhz);
#  ifdef ZgatewayRF
  disableRFReceive();
#  endif
#  ifdef ZgatewayPilight
  disablePilightReceive();
#  endif

#  ifdef ZradioCC1101
  ELECHOUSE_cc1101.SetRx(receiveMhz); // set Receive on
#  endif
  rtl_433.enableReceiver(RF_RECEIVER_GPIO);
  // rtl_433.initReceiver(RF_RECEIVER_GPIO);
  pinMode(RF_EMITTER_GPIO, OUTPUT); // Set this here, because if this is the RX pin it was reset to INPUT by Serial.end();
}

extern void disableRTLreceive() {
  Log.trace(F("disableRTLreceive" CR));
  rtl_433.enableReceiver(-1);
  rtl_433.disableReceiver();
}

#endif