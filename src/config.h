/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef CONFIG_NET_CONFIG_SETTINGS
#ifdef CONFIG_NET_IPV6
#define ZEPHYR_ADDR CONFIG_NET_CONFIG_MY_IPV6_ADDR
#define SERVER_ADDR CONFIG_NET_CONFIG_PEER_IPV6_ADDR
#else
#define ZEPHYR_ADDR CONFIG_NET_CONFIG_MY_IPV4_ADDR
#define SERVER_ADDR CONFIG_NET_CONFIG_PEER_IPV4_ADDR
#endif
#else
#ifdef CONFIG_NET_IPV6
#define ZEPHYR_ADDR "2001:db8::1"
#define SERVER_ADDR "2001:db8::2"
#else
#define ZEPHYR_ADDR "192.168.31.127"
#define SERVER_ADDR "192.168.31.215"
#endif
#endif

#if defined(CONFIG_MQTT_LIB_SOCKS)
#define SOCKS5_PROXY_ADDR SERVER_ADDR
#define SOCKS5_PROXY_PORT 1080
#endif

#ifdef CONFIG_MQTT_LIB_TLS
#define SERVER_PORT 8883
#else
#define SERVER_PORT 1883
#endif

#define APP_SLEEP_MSECS 500
#define APP_TX_RX_TIMEOUT 300
#define APP_NET_INIT_TIMEOUT 10000

#define APP_CONNECT_TRIES 10

#define APP_MAX_ITERATIONS 100

#define APP_MQTT_BUFFER_SIZE 128

#define MQTT_CLIENTID "zephyr_publisher"

/* Set the following to 1 to enable the Bluemix topic format */
#define APP_BLUEMIX_TOPIC 0

/* These are the parameters for the Bluemix topic format */
#if APP_BLUEMIX_TOPIC
#define BLUEMIX_DEVTYPE "sensor"
#define BLUEMIX_DEVID "carbon"
#define BLUEMIX_EVENT "status"
#define BLUEMIX_FORMAT "json"
#endif

// fan config
#define fan_controlledByMQTT
#define useMQTT

// QOS
#define QOS MQTT_QOS_1_AT_LEAST_ONCE

// initial pwm fan speed on startup (0 <= value <= 255)
#define INITIALPWMVALUE 120

// delta used when manually increasing or decreasing pwm
#define PWMSTEP 10

// fanTacho
#define TACHOUPDATECYCLE \
  1000  // how often tacho speed shall be determined, in milliseconds
#define NUMBEROFINTERRUPSINONESINGLEROTATION \
  2  // Number of interrupts ESP32 sees on tacho signal on a single fan
     // rotation. All the fans I've seen trigger two interrups.

// home assistant
#define useHomeassistantMQTTDiscovery

#define UNIQUE_DEVICE_FRIENDLYNAME "Fan Controller"
#define UNIQUE_DEVICE_NAME "fan_controller"

#define HASSSTATUSTOPIC "homeassistant/status" // can be "online" and "offline"
#define HASSSTATUSONLINEPAYLOAD "online"
#define HASSSTATUSOFFLINEPAYLOAD "offline"

#define HOMEASSISTANTDEVICE                         \
  "\"dev\":{\"name\":\"" UNIQUE_DEVICE_FRIENDLYNAME \
  "\", \"model\":\"" UNIQUE_DEVICE_NAME             \
  "\", \"identifiers\":[\"" UNIQUE_DEVICE_NAME      \
  "\"], \"manufacturer\":\"KlausMu\"}"

#define HASSFANDISCOVERYTOPIC "homeassistant/fan/" UNIQUE_DEVICE_NAME "/config"
#define HASSFANDISCOVERYPAYLOAD                                               \
  "{\"name\":null,            \"unique_id\":\"" UNIQUE_DEVICE_NAME            \
  "\",             \"object_id\":\"" UNIQUE_DEVICE_NAME                       \
  "\",             \"~\":\"" UNIQUE_DEVICE_NAME                               \
  "\", \"icon\":\"mdi:fan\", \"command_topic\":\"~/cmnd/MODE\", "             \
  "\"state_topic\":\"~/stat/MODE\", \"payload_on\": \"fan_only\", "           \
  "\"payload_off\": \"off\", \"percentage_state_topic\": \"~/stat/FANPWM\", " \
  "\"percentage_command_topic\": \"~/cmnd/FANPWM\", \"speed_range_min\": 1, " \
  "\"speed_range_max\": "                                                     \
  "255,\"availability_topic\":\"~/stat/STATUS\", " HOMEASSISTANTDEVICE "}"

#define HASSPWMSENSORDISCOVERYTOPIC \
  "homeassistant/sensor/" UNIQUE_DEVICE_NAME "/pwm/config"
#define HASSPWMSENSORDISCOVERYPAYLOAD                                          \
  "{\"name\":\"PWM\",         \"unique_id\":\"" UNIQUE_DEVICE_NAME             \
  "_PWM\",         \"object_id\":\"" UNIQUE_DEVICE_NAME                        \
  "_PWM\",         \"~\":\"" UNIQUE_DEVICE_NAME                                \
  "\", \"state_topic\":\"~/tele/STATE2\", \"value_template\":\"{{ "            \
  "value_json.pwm }}\",                                                      " \
  "                                      \"state_class\":\"measurement\", "    \
  "\"expire_after\": \"30\",                                                 " \
  "                                                                          " \
  "                                                                          " \
  "                                                                          " \
  "              " HOMEASSISTANTDEVICE "}"
#define HASSRPMSENSORDISCOVERYTOPIC \
  "homeassistant/sensor/" UNIQUE_DEVICE_NAME "/rpm/config"
#define HASSRPMSENSORDISCOVERYPAYLOAD                                          \
  "{\"name\":\"RPM\",         \"unique_id\":\"" UNIQUE_DEVICE_NAME             \
  "_RPM\",         \"object_id\":\"" UNIQUE_DEVICE_NAME                        \
  "_RPM\",         \"~\":\"" UNIQUE_DEVICE_NAME                                \
  "\", \"state_topic\":\"~/tele/STATE2\", \"value_template\":\"{{ "            \
  "value_json.rpm }}\",                                                      " \
  "                                      \"state_class\":\"measurement\", "    \
  "\"expire_after\": \"30\",                                                 " \
  "                                                                          " \
  "                                                                          " \
  "                                                                          " \
  "              " HOMEASSISTANTDEVICE "}"

#define WAITAFTERHAISONLINEUNTILDISCOVERYWILLBESENT 1000
#define HASSFANSTATUSTOPIC UNIQUE_DEVICE_NAME "/stat/STATUS"

#define MQTTCMNDTARGETTEMP UNIQUE_DEVICE_NAME "/cmnd/TARGETTEMP"
#define MQTTSTATTARGETTEMP UNIQUE_DEVICE_NAME "/stat/TARGETTEMP"
#define MQTTCMNDACTUALTEMP UNIQUE_DEVICE_NAME "/cmnd/ACTUALTEMP"
#define MQTTSTATACTUALTEMP UNIQUE_DEVICE_NAME "/stat/ACTUALTEMP"
#define MQTTCMNDFANPWM UNIQUE_DEVICE_NAME "/cmnd/FANPWM"
#define MQTTSTATFANPWM UNIQUE_DEVICE_NAME "/stat/FANPWM"
// https://www.home-assistant.io/integrations/climate.mqtt/#mode_command_topic
// https://www.home-assistant.io/integrations/climate.mqtt/#mode_state_topic
// note: it is not guaranteed that fan stops if pwm is set to 0
#define MQTTCMNDFANMODE                                                        \
  UNIQUE_DEVICE_NAME "/cmnd/MODE" // can be "off" and "fan_only"
#define MQTTSTATFANMODE UNIQUE_DEVICE_NAME "/stat/MODE"
#define MQTTFANMODEOFFPAYLOAD "off"
#define MQTTFANMODEFANONLYPAYLOAD "fan_only"

#if defined(useOTAUpdate)
#define MQTTCMNDOTA UNIQUE_DEVICE_NAME "/cmnd/OTA"
#endif

#ifdef useTemperatureSensorBME280
#define MQTTTELESTATE1 UNIQUE_DEVICE_NAME "/tele/STATE1"
#endif
#define MQTTTELESTATE2 UNIQUE_DEVICE_NAME "/tele/STATE2"
#define MQTTTELESTATE3 UNIQUE_DEVICE_NAME "/tele/STATE3"
#define MQTTTELESTATE4 UNIQUE_DEVICE_NAME "/tele/STATE4"


/////////////////////
// missing functions
#define millis() k_uptime_get_32()

#endif