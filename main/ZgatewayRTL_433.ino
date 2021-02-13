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

#include <rtl_433_ESP.h>

#define CC1101_FREQUENCY 433.92
#define JSON_MSG_BUFFER 512
#define ONBOARD_LED 2

char messageBuffer[JSON_MSG_BUFFER];

rtl_433_ESP rtl_433(-1); // use -1 to disable transmitter

#include <ELECHOUSE_CC1101_SRC_DRV.h>

void rtl_433_Callback(char *protocol, char *message, unsigned int modulation)
{
  StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer2;
  JsonObject &RFrtl_433_ESPdata = jsonBuffer2.parseObject(message);
  RFrtl_433_ESPdata.set("protocol", protocol);
  RFrtl_433_ESPdata.set("modulation", modulation);
  pub(subjectRTL_433toMQTT, RFrtl_433_ESPdata);
#ifdef MEMORY_DEBUG
  logprintfLn(LOG_INFO, "Post rtl_433_Callback: %d", ESP.getFreeHeap());
#endif
}

void rtl_433setup()
{
  rtl_433.initReceiver(RF_RECEIVER_GPIO, CC1101_FREQUENCY);
  rtl_433.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
  Log.trace(F("ZgatewayRTL_433 setup done " CR));
}

void rtl_433loop()
{
  rtl_433.loop();
}

#endif