#include <logging/log.h>
LOG_MODULE_DECLARE(fan_controller, LOG_LEVEL_DBG);

// user include
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "fanPWM.h"
#include "fanTacho.h"
#include "mqtt_helper.h"
#include "sensorBME280.h"
#include "temperatureController.h"
#include "tft.h"

#ifdef useMQTT
// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// https://github.com/knolleary/pubsubclient
// https://gist.github.com/igrr/7f7e7973366fc01d6393

unsigned long reconnectInterval = 5000;
// in order to do reconnect immediately ...
unsigned long lastReconnectAttempt = 0;
#ifdef useHomeassistantMQTTDiscovery
unsigned long timerStartForHAdiscovery = 1;
#endif

char *itoa(int num, char *str, int radix) { /*索引表*/
  char index[] = "0123456789ABCDEF";
  unsigned unum; /*中间变量*/
  int i = 0, j, k;
  /*确定unum的值*/
  if (radix == 10 && num < 0) /*十进制负数*/
  {
    unum = (unsigned)-num;
    str[i++] = '-';
  } else
    unum = (unsigned)num; /*其他情况*/
  /*转换*/
  do {
    str[i++] = index[unum % (unsigned)radix];
    unum /= radix;
  } while (unum);
  str[i] = '\0';
  /*逆序*/
  if (str[0] == '-')
    k = 1; /*十进制负数*/
  else
    k = 0;

  for (j = k; j <= (i - 1) / 2; j++) {
    char temp;
    temp = str[j];
    str[j] = str[i - 1 + k - j];
    str[i - 1 + k - j] = temp;
  }
  return str;
}

// zephyr mqtt

static void prepare_fds(struct mqtt_client *client) {
  if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
    fds[0].fd = client->transport.tcp.sock;
  }
#if defined(CONFIG_MQTT_LIB_TLS)
  else if (client->transport.type == MQTT_TRANSPORT_SECURE) {
    fds[0].fd = client->transport.tls.sock;
  }
#endif

  fds[0].events = ZSOCK_POLLIN;
  nfds = 1;
}

static void clear_fds(void) { nfds = 0; }

static void wait(int timeout) {
  if (nfds > 0) {
    if (poll(fds, nfds, timeout) < 0) {
      printk("poll error: %d\n", errno);
    }
  }
}

int process_mqtt_and_sleep(struct mqtt_client *client, int timeout) {
  s64_t remaining = timeout;
  s64_t start_time = k_uptime_get();
  int rc;

  while (remaining > 0 && connected) {
    wait(remaining);

    rc = mqtt_live(client);
    if (rc != 0) {
      PRINT_RESULT("mqtt_live", rc);
      return rc;
    }

    rc = mqtt_input(client);
    if (rc != 0) {
      PRINT_RESULT("mqtt_input", rc);
      return rc;
    }

    remaining = timeout + start_time - k_uptime_get();
  }

  return 0;
}

void client_init(struct mqtt_client *client) {
  mqtt_client_init(client);

  broker_init();

  /* MQTT client configuration */
  client->broker = &broker;
  client->evt_cb = mqtt_evt_handler;
  client->client_id.utf8 = (u8_t *)MQTT_CLIENTID;
  client->client_id.size = strlen(MQTT_CLIENTID);
  client->password = NULL;
  client->user_name = NULL;
  client->protocol_version = MQTT_VERSION_3_1_1;

  /* MQTT buffers configuration */
  client->rx_buf = rx_buffer;
  client->rx_buf_size = sizeof(rx_buffer);
  client->tx_buf = tx_buffer;
  client->tx_buf_size = sizeof(tx_buffer);

  /* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
  client->transport.type = MQTT_TRANSPORT_SECURE;

  struct mqtt_sec_config *tls_config = &client->transport.tls.config;

  tls_config->peer_verify = 2;
  tls_config->cipher_list = NULL;
  tls_config->sec_tag_list = m_sec_tags;
  tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
  tls_config->hostname = TLS_SNI_HOSTNAME;
#else
  tls_config->hostname = NULL;
#endif

#else
#if defined(CONFIG_MQTT_LIB_SOCKS)
  client->transport.type = MQTT_TRANSPORT_SOCKS;
  client->transport.socks5.proxy = &socks5_proxy;
#else
  client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
#endif
}

static void broker_init(void) {
#if defined(CONFIG_NET_IPV6)
  struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

  broker6->sin6_family = AF_INET6;
  broker6->sin6_port = htons(SERVER_PORT);
  inet_pton(AF_INET6, SERVER_ADDR, &broker6->sin6_addr);

#if defined(CONFIG_MQTT_LIB_SOCKS)
  struct sockaddr_in6 *proxy6 = (struct sockaddr_in6 *)&socks5_proxy;

  proxy6->sin6_family = AF_INET6;
  proxy6->sin6_port = htons(SOCKS5_PROXY_PORT);
  inet_pton(AF_INET6, SOCKS5_PROXY_ADDR, &proxy6->sin6_addr);
#endif
#else
  struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

  broker4->sin_family = AF_INET;
  broker4->sin_port = htons(SERVER_PORT);
  inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);

#if defined(CONFIG_MQTT_LIB_SOCKS)
  struct sockaddr_in *proxy4 = (struct sockaddr_in *)&socks5_proxy;

  proxy4->sin_family = AF_INET;
  proxy4->sin_port = htons(SOCKS5_PROXY_PORT);
  inet_pton(AF_INET, SOCKS5_PROXY_ADDR, &proxy4->sin_addr);
#endif
#endif
}

/* In this routine we block until the connected variable is 1 */
static int try_to_connect(struct mqtt_client *client) {
  int rc, i = 0;

  while (i++ < APP_CONNECT_TRIES && !connected) {
    client_init(client);

    rc = mqtt_connect(client);
    if (rc != 0) {
      PRINT_RESULT("mqtt_connect", rc);
      k_sleep(APP_SLEEP_MSECS);
      continue;
    }

    prepare_fds(client);

    wait(APP_SLEEP_MSECS);
    mqtt_input(client);

    if (!connected) {
      mqtt_abort(client);
    }
  }

  if (connected) {
    return 0;
  }

  return -EINVAL;
}

void callback(char *topic, char *payload, unsigned int length);

void mqtt_evt_handler(struct mqtt_client *const client,
                      const struct mqtt_evt *evt) {
  int err;
  struct mqtt_puback_param puback;

  switch (evt->type) {
    case MQTT_EVT_CONNACK:
      if (evt->result != 0) {
        printk("MQTT connect failed %d\n", evt->result);
        break;
      }

      connected = true;
      printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);

      break;

    case MQTT_EVT_DISCONNECT:
      printk("[%s:%d] MQTT client disconnected %d\n", __func__, __LINE__,
             evt->result);

      connected = false;
      clear_fds();

      break;

    case MQTT_EVT_PUBACK:
      if (evt->result != 0) {
        printk("MQTT PUBACK error %d\n", evt->result);
        break;
      }

      printk("[%s:%d] PUBACK packet id: %u\n", __func__, __LINE__,
             evt->param.puback.message_id);

      break;

    case MQTT_EVT_PUBREC:
      if (evt->result != 0) {
        printk("MQTT PUBREC error %d\n", evt->result);
        break;
      }

      printk("[%s:%d] PUBREC packet id: %u\n", __func__, __LINE__,
             evt->param.pubrec.message_id);

      const struct mqtt_pubrel_param rel_param = {
          .message_id = evt->param.pubrec.message_id};

      err = mqtt_publish_qos2_release(client, &rel_param);
      if (err != 0) {
        printk("Failed to send MQTT PUBREL: %d\n", err);
      }

      break;

    case MQTT_EVT_PUBCOMP:
      if (evt->result != 0) {
        printk("MQTT PUBCOMP error %d\n", evt->result);
        break;
      }

      printk("[%s:%d] PUBCOMP packet id: %u\n", __func__, __LINE__,
             evt->param.pubcomp.message_id);

      break;

    case MQTT_EVT_PUBLISH: {
      int topic_len = evt->param.publish.message.topic.topic.size;
      LOG_INF("topic len %d", topic_len);
      if (topic_len > 25) {
        LOG_ERR("too long topic %d",
                topic_len);
        break;
      } else {
        size_t i = 0;
        for (; i < topic_len; i++) {
          topic[i] = (char)evt->param.publish.message.topic.topic.utf8[i];
        }
        topic[i] = '\0';
      }
      int len = evt->param.publish.message.payload.len;
      if (len > 50) {
        LOG_ERR("too long payload len %d", len);
        break;
      }
      int bytes_read;
      LOG_INF("MQTT publish received %d, %d bytes", evt->result, len);
      // LOG_INF("topic %s", topic);
      LOG_INF("   id: %d, qos: %d", evt->param.publish.message_id,
              evt->param.publish.message.topic.qos);

      while (len) {
        bytes_read = mqtt_read_publish_payload(&client_ctx, packet, len);
        if (bytes_read < 0 && bytes_read != -EAGAIN) {
          LOG_ERR("failure to read payload");
          break;
        }
        if (bytes_read < len) {
          LOG_ERR("not read payload in one read read:%d all:%d", bytes_read,
                  len);
          break;
        }
      }
      packet[len] = '\0';
      if (evt->param.publish.message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
        puback.message_id = evt->param.publish.message_id;
        mqtt_publish_qos1_ack(&client_ctx, &puback);
      }
      callback(topic, packet, len);
      break;
    }

    default:
      break;
  }
}

void mqtt_setup() {
  // #ifdef useHomeassistantMQTTDiscovery
  //   // Set buffer size to allow hass discovery payload
  //   mqttClient.setBufferSize(1280);
  // #endif
  return;
}

void mqtt_loop() {
  if (connected) {
    unsigned long currentMillis = millis();
    if ((currentMillis - lastReconnectAttempt) > reconnectInterval) {
      lastReconnectAttempt = currentMillis;
      // Attempt to reconnect
      checkMQTTconnection();
    }
  }

  if (connected) {
    process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
  }
}

bool checkMQTTconnection() {
  if (connected) {
    return true;
  } else {
    // try to connect to mqtt server
    // In case of Home Assistant, connect with last will to the broker to set
    // the device offline when the esp32 fan controller is swtiched off
    if (!try_to_connect(&client_ctx)) {
      LOG_INF("Successfully connected to MQTT broker");

      // subscribes to messages with given topic.
      // Callback function will be called 1. in client.loop() 2. when sending
      // a message
      int err = 0;
      // 1.
      subs_topic.topic.utf8 = MQTTCMNDTARGETTEMP;
      // -1 because dif between sizeof and strlen
      subs_topic.topic.size = sizeof(MQTTCMNDACTUALTEMP) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, MQTTCMNDTARGETTEMP);
      }
      // 2.
      subs_topic.topic.utf8 = MQTTCMNDACTUALTEMP;
      subs_topic.topic.size = sizeof(MQTTCMNDACTUALTEMP) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, MQTTCMNDACTUALTEMP);
      }
      // 3.
      subs_topic.topic.utf8 = MQTTCMNDFANPWM;
      subs_topic.topic.size = sizeof(MQTTCMNDFANPWM) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, MQTTCMNDFANPWM);
      }
      // 4.
      subs_topic.topic.utf8 = MQTTCMNDFANMODE;
      subs_topic.topic.size = sizeof(MQTTCMNDFANMODE) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, MQTTCMNDFANMODE);
      }

      // mqttClient.subscribe(MQTTCMNDTARGETTEMP);
      // mqttClient.subscribe(MQTTCMNDACTUALTEMP);
      // mqttClient.subscribe(MQTTCMNDFANPWM);
      // mqttClient.subscribe(MQTTCMNDFANMODE);
#if defined(useHomeassistantMQTTDiscovery)
      // 5.
      subs_topic.topic.utf8 = HASSSTATUSTOPIC;
      subs_topic.topic.size = sizeof(HASSSTATUSTOPIC) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, HASSSTATUSTOPIC);
      }
      // mqttClient.subscribe(HASSSTATUSTOPIC);
      //  if we successfully connected or reconnected to the mqtt server, send
      //  HA discovery
      timerStartForHAdiscovery = millis();
#endif
#if defined(useOTAUpdate)
      // 6.
      subs_topic.topic.utf8 = MQTTCMNDOTA;
      subs_topic.topic.size = sizeof(MQTTCMNDOTA) - 1;
      subs_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
      subs_list.list = &subs_topic;
      subs_list.list_count = 1;
      subs_list.message_id = current_msg_id++;
      err = mqtt_subscribe(&client_ctx, &subs_list);
      if (err) {
        LOG_ERR("Failed to subscribe %d topic: %s", err, MQTTCMNDACTUALTEMP);
      }
      // mqttClient.subscribe(MQTTCMNDOTA);
#endif
    } else {
      LOG_INF(
          "  MQTT connection failed. Will try later "
          "...");
      return false;
    }
  }
}

bool publishMQTTMessage(const char *topic, const char *payload) {
  if (checkMQTTconnection()) {
    //  LOG_INF("Sending mqtt payload to topic \"%s\": %s", topic, payload);
    struct mqtt_publish_param param;

    param.message.topic.qos = QOS;
    param.message.topic.topic.utf8 = (u8_t *)topic;
    param.message.topic.topic.size = strlen(topic);
    param.message.payload.data = payload;
    param.message.payload.len = strlen(payload);
    param.message_id = current_msg_id++;
    param.dup_flag = 0;
    param.retain_flag = 0;
    if (!mqtt_publish(&client_ctx, &param)) {
      // LOG_INF("Publish ok");
      return true;
    } else {
      LOG_INF("Publish failed");
    }
  } else {
    LOG_INF(
        "  Cannot publish mqtt message, because checkMQTTconnection failed "
        "(WiFi or mqtt is not connected)");
  }
  return false;
}

bool mqtt_publish_stat_targetTemp() {
  char str[10];
  sprintf(str, "%.2f", getTargetTemperature());
  return publishMQTTMessage(MQTTSTATTARGETTEMP, str);
};
bool mqtt_publish_stat_actualTemp() {
  char str[10];
  sprintf(str, "%.2f", getActualTemperature());
  return publishMQTTMessage(MQTTSTATACTUALTEMP, str);
};
bool mqtt_publish_stat_fanPWM() {
  char str[10];
  sprintf(str, "%.2f", getPWMvalue());
  return publishMQTTMessage(MQTTSTATFANPWM, str);
};
bool mqtt_publish_stat_mode() {
  return publishMQTTMessage(MQTTSTATFANMODE, getModeIsOff()
                                                 ? MQTTFANMODEOFFPAYLOAD
                                                 : MQTTFANMODEFANONLYPAYLOAD);
};
#ifdef useShutdownButton
bool mqtt_publish_shutdown() {
  return publishMQTTMessage(MQTTCMNDSHUTDOWNTOPIC, MQTTCMNDSHUTDOWNPAYLOAD);
};
#endif

#ifdef useHomeassistantMQTTDiscovery
bool mqtt_publish_hass_discovery() {
  LOG_INF("Will send HA discovery now.");
  bool error = false;
#ifdef useAutomaticTemperatureControl
  error = !publishMQTTMessage(HASSCLIMATEDISCOVERYTOPIC,
                              HASSCLIMATEDISCOVERYPAYLOAD);
#else
  error = !publishMQTTMessage(HASSFANDISCOVERYTOPIC, HASSFANDISCOVERYPAYLOAD);
#endif
#ifdef useTemperatureSensorBME280
  error = error || !publishMQTTMessage(HASSHUMIDITYSENSORDISCOVERYTOPIC,
                                       HASSHUMIDITYSENSORDISCOVERYPAYLOAD);
  error = error || !publishMQTTMessage(HASSTEMPERATURESENSORDISCOVERYTOPIC,
                                       HASSTEMPERATURESENSORDISCOVERYPAYLOAD);
  error = error || !publishMQTTMessage(HASSPRESSURESENSORDISCOVERYTOPIC,
                                       HASSPRESSURESENSORDISCOVERYPAYLOAD);
  error = error || !publishMQTTMessage(HASSALTITUDESENSORDISCOVERYTOPIC,
                                       HASSALTITUDESENSORDISCOVERYPAYLOAD);
#endif
  error = error || !publishMQTTMessage(HASSPWMSENSORDISCOVERYTOPIC,
                                       HASSPWMSENSORDISCOVERYPAYLOAD);
  error = error || !publishMQTTMessage(HASSRPMSENSORDISCOVERYTOPIC,
                                       HASSRPMSENSORDISCOVERYPAYLOAD);

  if (!error) {
    // delay(1000);
    k_sleep(1000);
  }
  // publish that we are online. Remark: offline is sent via last will
  // retained message
  error = error || !publishMQTTMessage(HASSFANSTATUSTOPIC, "");
  error =
      error || !publishMQTTMessage(HASSFANSTATUSTOPIC, HASSSTATUSONLINEPAYLOAD);

  if (!error) {
    // delay(1000);
    k_sleep(1000);
  }
  // MODE????

  // that's not really part of the discovery message, but this enables the
  // climate slider in HA and immediately provides all values
  error = error || !mqtt_publish_stat_targetTemp();
  error = error || !mqtt_publish_stat_actualTemp();
  error = error || !mqtt_publish_stat_fanPWM();
  error = error || !mqtt_publish_stat_mode();
  error = error || !mqtt_publish_tele();
  if (!error) {
    // will not resend discovery as long as timerStartForHAdiscovery == 0
    LOG_INF(
        "Will set timer to 0 now, this means I will not send discovery "
        "again.");
    timerStartForHAdiscovery = 0;
  } else {
    LOG_INF("Some error occured while sending discovery. Will try again.");
  }
  return !error;
}
#endif

bool mqtt_publish_tele() {
  bool error = false;
  // maximum message length 128 Byte
  char payload[64] = "";
// BME280
#ifdef useTemperatureSensorBME280
  payload += "{\"ActTemp\":";
  payload += lastTempSensorValues[0];
  payload += ",\"pres\":";
  payload += lastTempSensorValues[1];
  payload += ",\"alt\":";
  payload += lastTempSensorValues[2];
  payload += ",\"hum\":";
  payload += lastTempSensorValues[3];
  payload += ",\"TargTemp\":";
  payload += getTargetTemperature();
  payload += "}";
  error = !publishMQTTMessage(MQTTTELESTATE1, payload.c_str());
#endif

  // Fan
  char str[10];
  memset(payload, '\0', sizeof(payload));
  strcat(payload, "{\"rpm\":");
  strcat(payload, itoa(last_rpm, str, 10));
  memset(str, '\0', sizeof(str));
  strcat(payload, ",\"pwm\":");
  strcat(payload, itoa(getPWMvalue(), str, 10));
  strcat(payload, "}");
  error = error || !publishMQTTMessage(MQTTTELESTATE2, payload);

  // WiFi
  // payload = "";
  // payload += "{\"wifiRSSI\":";
  // payload += WiFi.RSSI();
  // payload += ",\"wifiChan\":";
  // payload += WiFi.channel();
  // payload += ",\"wifiSSID\":";
  // payload += WiFi.SSID();
  // payload += ",\"wifiBSSID\":";
  // payload += WiFi.BSSIDstr();
  // #if defined(WIFI_KNOWN_APS)
  // payload += ",\"wifiAP\":";
  // payload += accessPointName;
  // #endif
  // payload += ",\"IP\":";
  // payload += WiFi.localIP().toString();
  // payload += "}";
  // error = error || !publishMQTTMessage(MQTTTELESTATE3, payload.c_str());

  // ESP32 stats
  // payload = "";
  // payload += "{\"up\":";
  // payload += String(millis());
  // payload += ",\"heapSize\":";
  // payload += String(ESP.getHeapSize());
  // payload += ",\"heapFree\":";
  // payload += String(ESP.getFreeHeap());
  // payload += ",\"heapMin\":";
  // payload += String(ESP.getMinFreeHeap());
  // payload += ",\"heapMax\":";
  // payload += String(ESP.getMaxAllocHeap());
  // payload += "}";
  // error = error || !publishMQTTMessage(MQTTTELESTATE4, payload.c_str());

  return !error;
}

void callback(char *topic, char *payload, unsigned int length) {
  // handle message arrived
  // std::string strPayload(reinterpret_cast<const char *>(payload), length);

  LOG_INF("MQTT message arrived [%s] %s", topic, payload);

  if (!strcmp(topic, MQTTCMNDTARGETTEMP)) {
#ifdef useAutomaticTemperatureControl
    LOG_INF("Setting targetTemp via mqtt");
    float num_float = ::atof(strPayload.c_str());
    LOG_INF("new targetTemp: %.2f", num_float);
    updatePWM_MQTT_Screen_withNewTargetTemperature(num_float, true);
#else
    LOG_INF(
        "\"#define useAutomaticTemperatureControl\" is NOT used in config.h "
        "Cannot set target temperature. Please set fan pwm.");
    updatePWM_MQTT_Screen_withNewTargetTemperature(getTargetTemperature(),
                                                   true);
#endif
  } else if (!strcmp(topic, MQTTCMNDACTUALTEMP)) {
#if defined(useAutomaticTemperatureControl) && \
    defined(setActualTemperatureViaMQTT)
    LOG_INF("Setting actualTemp via mqtt");
    float num_float = ::atoi(strPayload.c_str());
    LOG_INF("new actualTemp: %.2f", num_float);
    updatePWM_MQTT_Screen_withNewActualTemperature(num_float, true);
#else
    LOG_INF(
        "\"#define setActualTemperatureViaMQTT\" is NOT used in config.h  "
        "Cannot set actual temperature. Please use BME280.");
    updatePWM_MQTT_Screen_withNewActualTemperature(getActualTemperature(),
                                                   true);
#endif
  } else if (!strcmp(topic, MQTTCMNDFANPWM)) {
#ifndef useAutomaticTemperatureControl
    LOG_INF("Setting fan pwm via mqtt");
    int num_int = atoi(payload);
    LOG_INF("new fan pwm: %d", num_int);
    updateMQTT_Screen_withNewPWMvalue(num_int, true);
#else
    LOG_INF(
        "\"#define useAutomaticTemperatureControl\" is used in config.h  "
        "Cannot set fan pwm. Please set target temperature.");
    updateMQTT_Screen_withNewPWMvalue(getPWMvalue(), true);
#endif
  } else if (!strcmp(topic, MQTTCMNDFANMODE)) {
    LOG_INF("Setting HVAC mode from HA received via mqtt");
    if (!strcmp(payload, MQTTFANMODEFANONLYPAYLOAD)) {
      LOG_INF("  Will turn fan into \"fan_only\" mode");
      updateMQTT_Screen_withNewMode(false, true);
    } else if (!strcmp(payload, MQTTFANMODEOFFPAYLOAD)) {
      LOG_INF("  Will switch fan off");
      updateMQTT_Screen_withNewMode(true, true);
    } else {
      LOG_INF("Payload %s not supported", payload);
    }
#if defined(useOTAUpdate)
  } else if (topicReceived == topicCmndOTA) {
    if (strPayload == "ON") {
      LOG_INF("MQTT command TURN ON OTA received");
      ArduinoOTA.begin();
    } else if (strPayload == "OFF") {
      LOG_INF("MQTT command TURN OFF OTA received");
      ArduinoOTA.end();
    } else {
      LOG_INF("Payload %s not supported", payload);
    }
#endif
#if defined(useHomeassistantMQTTDiscovery)
  } else if (!strcmp(topic, HASSSTATUSTOPIC)) {
    if (!strcmp(payload, HASSSTATUSONLINEPAYLOAD)) {
      LOG_INF(
          "HA status online received. This means HA has restarted. Will send "
          "discovery again in some seconds as defined in config.h");
      // set timer so that discovery will be resent after some seconds (as
      // defined in config.h)
      timerStartForHAdiscovery = millis();
      // Very unlikely. Can only happen if millis() overflowed max unsigned
      // long every approx. 50 days
      if (timerStartForHAdiscovery == 0) {
        timerStartForHAdiscovery = 1;
      }
    } else if (!strcmp(payload, HASSSTATUSOFFLINEPAYLOAD)) {
      LOG_INF(
          "HA status offline received. Nice to know. Currently we don't "
          "react "
          "to this.");
    } else {
      LOG_INF("Payload %s not supported", payload);
    }
#endif
  }
}
#endif
