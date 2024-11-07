#include <net/mqtt.h>
#include <net/socket.h>
#include <zephyr.h>

#include "config.h"

/* Buffers for MQTT client. */
static u8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static u8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

static char packet[51];
static char topic[25];

/* The mqtt client struct */
#ifdef useHomeassistantMQTTDiscovery
static struct mqtt_topic will_topic_qos_0 = {
    .qos = 0,
    .topic.utf8 = HASSFANSTATUSTOPIC,
    .topic.size = sizeof(HASSFANSTATUSTOPIC) - 1};

static struct mqtt_utf8 will_msg = {
    .utf8 = HASSSTATUSOFFLINEPAYLOAD,
    .size = sizeof(HASSSTATUSOFFLINEPAYLOAD) - 1};

static struct mqtt_client client_ctx = {.will_retain = 1,
                                        .will_topic = &will_topic_qos_0,
                                        .will_message = &will_msg,
                                        .user_name = NULL,
                                        .password = NULL};
#else
static struct mqtt_client client_ctx;
#endif

/* MQTT Broker details. */
static struct sockaddr_storage broker;
#if defined(CONFIG_MQTT_LIB_SOCKS)
static struct sockaddr_storage socks5_proxy;
#endif

static struct pollfd fds[1];
static int nfds;

static bool connected;

// 0 will cause failure
static int current_msg_id = 1;

static struct mqtt_topic subs_topic;
static struct mqtt_subscription_list subs_list;

#define SUCCESS_OR_EXIT(rc) \
  {                         \
    if (rc != 0) {          \
      return;               \
    }                       \
  }
#define SUCCESS_OR_BREAK(rc) \
  {                          \
    if (rc != 0) {           \
      break;                 \
    }                        \
  }
#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
  printk("[%s:%d] %s: %d <%s>\n", __func__, __LINE__, (func), rc, RC_STR(rc))

static void broker_init(void);
static void client_init(struct mqtt_client *client);
static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout);
static void mqtt_evt_handler(struct mqtt_client *const client,
                             const struct mqtt_evt *evt);

#ifdef useMQTT
bool publishMQTTMessage(const char *topic, const char *payload);
bool checkMQTTconnection();
void mqtt_setup(void);
void mqtt_loop(void);
bool mqtt_publish_tele(void);
bool mqtt_publish_stat_targetTemp();
bool mqtt_publish_stat_actualTemp();
bool mqtt_publish_stat_fanPWM();
bool mqtt_publish_stat_mode();
#ifdef useShutdownButton
bool mqtt_publish_shutdown();
#endif
#ifdef useHomeassistantMQTTDiscovery
/* Sets the start of the timer until HA discovery is sent.
   It will be waited WAITAFTERHAISONLINEUNTILDISCOVERYWILLBESENT ms before the
   discovery is sent. 0: discovery will not be sent >0: discovery will be sent
   as soon as "WAITAFTERHAISONLINEUNTILDISCOVERYWILLBESENT" ms are over
*/
extern unsigned long timerStartForHAdiscovery;
bool mqtt_publish_hass_discovery();

// itoa function
char *itoa(int num, char *str, int radix);
#endif
#endif