#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"

#include "src/shared/util.h"
#include "src/shared/queue.h"
#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-client.h"
#include "src/shared/mainloop.h"

#include "config.h"

#define ATT_CID 4

#define UUID_ESS_SERVICE 0x181A
#define UUID_TEMPERATURE 0x2A6E
#define UUID_PRESSURE 0x2A6D
#define UUID_HUMIDITY 0x2A6F

#ifndef VERBOSE
#define VERBOSE 0
#endif

struct client {
    int fd;
    struct bt_att* att;
    struct gatt_db* db;
    struct bt_gatt_client* gatt;

    unsigned int reliable_session_id;
};

struct ble_sensor_state {
    bool connected;

    bool has_temp;
    bool has_press;
    bool has_humid;

    float temperature;
    float pressure;
    float humidity;

    pthread_mutex_t lock;
};

static struct ble_sensor_state g_state = {.connected = false,
                                          .has_temp = false,
                                          .has_press = false,
                                          .has_humid = false,
                                          .temperature = 0.0f,
                                          .pressure = 0.0f,
                                          .humidity = 0.0f,
                                          .lock = PTHREAD_MUTEX_INITIALIZER};

static bdaddr_t g_dst_addr;
static uint8_t g_dst_type = BDADDR_LE_RANDOM;
static int g_sec = BT_SECURITY_LOW;
static uint16_t g_mtu = 0;
static struct client* g_cli = NULL;

/* notification */
static void notify_cb(uint16_t value_handle,
                      const uint8_t* value,
                      uint16_t length,
                      void* user_data);
static void register_notify_cb(uint16_t att_ecode, void* user_data);

/* connection */
static void reconnect_cb(int id, void* user_data);
static void att_disconnect_cb(int err, void* user_data);

/* logs */
static void log_service_event(struct gatt_db_attribute* attr, const char* str);
static void service_added_cb(struct gatt_db_attribute* attr, void* user_data);
static void service_removed_cb(struct gatt_db_attribute* attr, void* user_data);
static void att_debug_cb(const char* str, void* user_data);
static void gatt_debug_cb(const char* str, void* user_data);

/* print helpers */
static void print_uuid(const bt_uuid_t* uuid);
static void print_incl(struct gatt_db_attribute* attr, void* user_data);
static void print_desc(struct gatt_db_attribute* attr, void* user_data);
static void print_chrc(struct gatt_db_attribute* attr, void* user_data);
static void print_service(struct gatt_db_attribute* attr, void* user_data);
static void print_services(struct client* cli);

/* callbacks */
static void ess_char_cb(struct gatt_db_attribute* attr, void* user_data);
static void service_cb(struct gatt_db_attribute* attr, void* user_data);
static void ready_cb(bool success, uint8_t att_ecode, void* user_data);
static void service_changed_cb(uint16_t start_handle,
                               uint16_t end_handle,
                               void* user_data);

/* lifecycle */
static struct client* client_create(int fd, uint16_t mtu);
static void client_destroy(struct client* cli);

static void signal_cb(int signum, void* user_data);

static int l2cap_le_att_connect(bdaddr_t* src,
                                bdaddr_t* dst,
                                uint8_t dst_type,
                                int sec);

/* public API */
bool ble_client_start(void);
void ble_client_stop(void);
bool ble_get_temperature(float* out);
bool ble_get_pressure(float* out);
bool ble_get_humidity(float* out);
bool ble_is_connected(void);

static void notify_cb(uint16_t value_handle,
                      const uint8_t* value,
                      uint16_t length,
                      void* user_data) {
    struct client* cli = user_data;
    struct gatt_db_attribute* attr;
    bt_uuid_t uuid;

    uint16_t handle, vh;
    uint8_t props;
    uint16_t ext_props;

    /* Look up attribute by handle */
    attr = gatt_db_get_attribute(cli->db, value_handle);
    if (!attr)
        return;

    /* Extract characteristic UUID */
    if (!gatt_db_attribute_get_char_data(attr, &handle, &vh, &props, &ext_props,
                                         &uuid))
        return;

    bt_uuid_t temp_uuid, press_uuid, humid_uuid;
    bt_uuid16_create(&temp_uuid, UUID_TEMPERATURE);
    bt_uuid16_create(&press_uuid, UUID_PRESSURE);
    bt_uuid16_create(&humid_uuid, UUID_HUMIDITY);

    if (bt_uuid_cmp(&uuid, &temp_uuid) == 0) {
        if (length < sizeof(int16_t))
            return;
        int16_t raw = le16toh(*(int16_t*)value);
        printf("Temperature: %.2f °C\n", raw / 100.0);

    } else if (bt_uuid_cmp(&uuid, &press_uuid) == 0) {
        if (length < sizeof(uint32_t))
            return;
        uint32_t raw = le32toh(*(uint32_t*)value);
        printf("Pressure: %.1f hPa\n", raw / 100.0);

    } else if (bt_uuid_cmp(&uuid, &humid_uuid) == 0) {
        if (length < sizeof(uint16_t))
            return;
        uint16_t raw = le16toh(*(uint16_t*)value);
        printf("Humidity: %.2f %%RH\n", raw / 100.0);
    }
}

static void register_notify_cb(uint16_t att_ecode, void* user_data) {
    if (att_ecode) {
        printf(
            "Failed to register notify handler "
            "- error code: 0x%02x\n",
            att_ecode);
        return;
    }
}

/* inner functions of the client */
static void reconnect_cb(int id, void* user_data) {
    int fd;
    struct client* cli = user_data;

    printf("Reconnecting...\n");

    fd = l2cap_le_att_connect(BDADDR_ANY, &g_dst_addr, g_dst_type, g_sec);

    if (fd < 0) {
        printf("Reconnect failed, retrying in 2s\n");
        mainloop_add_timeout(2000, reconnect_cb, cli, NULL);
        return;
    }

    client_create(fd, g_mtu);
}

static void att_disconnect_cb(int err, void* user_data) {
    struct client* cli = user_data;

    printf("Disconnected (%s)\n", strerror(err));

    client_destroy(cli);

    mainloop_add_timeout(1000, reconnect_cb, NULL, NULL);
}

static void log_service_event(struct gatt_db_attribute* attr, const char* str) {
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid;
    uint16_t start, end;

    gatt_db_attribute_get_service_uuid(attr, &uuid);
    bt_uuid_to_string(&uuid, uuid_str, sizeof(uuid_str));

    gatt_db_attribute_get_service_handles(attr, &start, &end);

    printf("%s - UUID: %s start: 0x%04x end: 0x%04x\n", str, uuid_str, start,
           end);
}

static void service_added_cb(struct gatt_db_attribute* attr, void* user_data) {
    log_service_event(attr, "Service Added");
}

static void service_removed_cb(struct gatt_db_attribute* attr,
                               void* user_data) {
    log_service_event(attr, "Service Removed");
}

static void att_debug_cb(const char* str, void* user_data) {
    const char* prefix = user_data;

    printf("%s %s\n", prefix, str);
}

static void gatt_debug_cb(const char* str, void* user_data) {
    const char* prefix = user_data;

    printf("%s%s\n", prefix, str);
}

static void print_uuid(const bt_uuid_t* uuid) {
    char uuid_str[MAX_LEN_UUID_STR];
    bt_uuid_t uuid128;

    bt_uuid_to_uuid128(uuid, &uuid128);
    bt_uuid_to_string(&uuid128, uuid_str, sizeof(uuid_str));

    printf("%s\n", uuid_str);
}

static void print_incl(struct gatt_db_attribute* attr, void* user_data) {
    struct client* cli = user_data;
    uint16_t handle, start, end;
    struct gatt_db_attribute* service;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_incl_data(attr, &handle, &start, &end))
        return;

    service = gatt_db_get_attribute(cli->db, start);
    if (!service)
        return;

    gatt_db_attribute_get_service_uuid(service, &uuid);

    printf(
        "\t  "
        "include"
        " - handle: "
        "0x%04x, - start: 0x%04x, end: 0x%04x,"
        "uuid: ",
        handle, start, end);
    print_uuid(&uuid);
}

static void print_desc(struct gatt_db_attribute* attr, void* user_data) {
    printf(
        "\t\t  "
        "descr"
        " - handle: 0x%04x, uuid: ",
        gatt_db_attribute_get_handle(attr));
    print_uuid(gatt_db_attribute_get_type(attr));
}

static void print_chrc(struct gatt_db_attribute* attr, void* user_data) {
    uint16_t handle, value_handle;
    uint8_t properties;
    uint16_t ext_prop;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_char_data(attr, &handle, &value_handle,
                                         &properties, &ext_prop, &uuid))
        return;

    printf(
        "\t  "
        "charac"
        " - start: 0x%04x, value: 0x%04x, "
        "props: 0x%02x, ext_props: 0x%04x, uuid: ",
        handle, value_handle, properties, ext_prop);
    print_uuid(&uuid);

    gatt_db_service_foreach_desc(attr, print_desc, NULL);
}

static void print_service(struct gatt_db_attribute* attr, void* user_data) {
    struct client* cli = user_data;
    uint16_t start, end;
    bool primary;
    bt_uuid_t uuid;

    if (!gatt_db_attribute_get_service_data(attr, &start, &end, &primary,
                                            &uuid))
        return;

    printf(
        "service"
        " - start: 0x%04x, "
        "end: 0x%04x, type: %s, uuid: ",
        start, end, primary ? "primary" : "secondary");
    print_uuid(&uuid);

    gatt_db_service_foreach_incl(attr, print_incl, cli);
    gatt_db_service_foreach_char(attr, print_chrc, NULL);

    printf("\n");
}

static void print_services(struct client* cli) {
    printf("\n");

    gatt_db_foreach_service(cli->db, NULL, print_service, cli);
}

static void ess_char_cb(struct gatt_db_attribute* attr, void* user_data) {
    struct client* cli = user_data;
    bt_uuid_t uuid;
    uint16_t value_handle;

    if (!gatt_db_attribute_get_char_data(attr, NULL, &value_handle, NULL, NULL,
                                         &uuid))
        return;

    bt_uuid_t temp_uuid, press_uuid, humid_uuid;
    bt_uuid16_create(&temp_uuid, UUID_TEMPERATURE);
    bt_uuid16_create(&press_uuid, UUID_PRESSURE);
    bt_uuid16_create(&humid_uuid, UUID_HUMIDITY);

    if (bt_uuid_cmp(&uuid, &temp_uuid) == 0 ||
        bt_uuid_cmp(&uuid, &press_uuid) == 0 ||
        bt_uuid_cmp(&uuid, &humid_uuid) == 0) {
        printf("Registering notify for handle 0x%04x\n", value_handle);

        bt_gatt_client_register_notify(
            cli->gatt, value_handle, register_notify_cb, notify_cb, cli, NULL);
    }
}

static void service_cb(struct gatt_db_attribute* attr, void* user_data) {
    struct client* cli = user_data;
    bt_uuid_t uuid;
    bt_uuid_t ess_uuid;

    if (!gatt_db_attribute_get_service_uuid(attr, &uuid))
        return;

    bt_uuid16_create(&ess_uuid, UUID_ESS_SERVICE);

    if (bt_uuid_cmp(&uuid, &ess_uuid) != 0)
        return;

    printf("ESS service found\n");

    gatt_db_service_foreach_char(attr, ess_char_cb, cli);
}

static void ready_cb(bool success, uint8_t att_ecode, void* user_data) {
    struct client* cli = user_data;

    if (!success) {
        printf("GATT discovery failed (0x%02x)\n", att_ecode);
        return;
    }

    printf("GATT discovery complete\n");

    gatt_db_foreach_service(cli->db, NULL, service_cb, cli);
}

static void service_changed_cb(uint16_t start_handle,
                               uint16_t end_handle,
                               void* user_data) {
    struct client* cli = user_data;

    printf("\nService Changed handled - start: 0x%04x end: 0x%04x\n",
           start_handle, end_handle);

    gatt_db_foreach_service_in_range(cli->db, NULL, print_service, cli,
                                     start_handle, end_handle);
}

static struct client* client_create(int fd, uint16_t mtu) {
    struct client* cli;

    cli = new0(struct client, 1);
    if (!cli) {
        fprintf(stderr, "Failed to allocate memory for client\n");
        return NULL;
    }

    cli->att = bt_att_new(fd, false);
    if (!cli->att) {
        fprintf(stderr, "Failed to initialze ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_set_close_on_unref(cli->att, true)) {
        fprintf(stderr, "Failed to set up ATT transport layer\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    if (!bt_att_register_disconnect(cli->att, att_disconnect_cb, NULL, NULL)) {
        fprintf(stderr, "Failed to set ATT disconnect handler\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->fd = fd;
    cli->db = gatt_db_new();
    if (!cli->db) {
        fprintf(stderr, "Failed to create GATT database\n");
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    cli->gatt = bt_gatt_client_new(cli->db, cli->att, mtu);
    if (!cli->gatt) {
        fprintf(stderr, "Failed to create GATT client\n");
        gatt_db_unref(cli->db);
        bt_att_unref(cli->att);
        free(cli);
        return NULL;
    }

    gatt_db_register(cli->db, service_added_cb, service_removed_cb, NULL, NULL);

    if (VERBOSE) {
        bt_att_set_debug(cli->att, att_debug_cb, "att: ", NULL);
        bt_gatt_client_set_debug(cli->gatt, gatt_debug_cb, "gatt: ", NULL);
    }

    bt_gatt_client_ready_register(cli->gatt, ready_cb, cli, NULL);
    bt_gatt_client_set_service_changed(cli->gatt, service_changed_cb, cli,
                                       NULL);

    /* bt_gatt_client already holds a reference */
    gatt_db_unref(cli->db);

    return cli;
}

static void client_destroy(struct client* cli) {
    bt_gatt_client_unref(cli->gatt);
    bt_att_unref(cli->att);
    free(cli);
}

static void signal_cb(int signum, void* user_data) {
    switch (signum) {
        case SIGINT:
        case SIGTERM:
            mainloop_quit();
            break;
        default:
            break;
    }
}

static int l2cap_le_att_connect(bdaddr_t* src,
                                bdaddr_t* dst,
                                uint8_t dst_type,
                                int sec) {
    int sock;
    struct sockaddr_l2 srcaddr, dstaddr;
    struct bt_security btsec;

    if (VERBOSE) {
        char srcaddr_str[18], dstaddr_str[18];

        ba2str(src, srcaddr_str);
        ba2str(dst, dstaddr_str);

        printf(
            "btgatt-client: Opening L2CAP LE connection on ATT "
            "channel:\n\t src: %s\n\tdest: %s\n",
            srcaddr_str, dstaddr_str);
    }

    sock = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) {
        perror("Failed to create L2CAP socket");
        return -1;
    }

    /* Set up source address */
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.l2_family = AF_BLUETOOTH;
    srcaddr.l2_cid = htobs(ATT_CID);
    srcaddr.l2_bdaddr_type = 0;
    bacpy(&srcaddr.l2_bdaddr, src);

    if (bind(sock, (struct sockaddr*)&srcaddr, sizeof(srcaddr)) < 0) {
        perror("Failed to bind L2CAP socket");
        close(sock);
        return -1;
    }

    /* Set the security level */
    memset(&btsec, 0, sizeof(btsec));
    btsec.level = sec;
    if (setsockopt(sock, SOL_BLUETOOTH, BT_SECURITY, &btsec, sizeof(btsec)) !=
        0) {
        fprintf(stderr, "Failed to set L2CAP security level\n");
        close(sock);
        return -1;
    }

    /* Set up destination address */
    memset(&dstaddr, 0, sizeof(dstaddr));
    dstaddr.l2_family = AF_BLUETOOTH;
    dstaddr.l2_cid = htobs(ATT_CID);
    dstaddr.l2_bdaddr_type = dst_type;
    bacpy(&dstaddr.l2_bdaddr, dst);

    printf("Connecting to device...");
    fflush(stdout);

    if (connect(sock, (struct sockaddr*)&dstaddr, sizeof(dstaddr)) < 0) {
        perror(" Failed to connect");
        close(sock);
        return -1;
    }

    printf(" Done\n");

    return sock;
}

/* public API */
bool ble_client_start(void) {
    bdaddr_t src_addr;
    bacpy(&src_addr, BDADDR_ANY);

    str2ba(BLE_MAC_STR, &g_dst_addr);

    mainloop_init();

    int fd = l2cap_le_att_connect(&src_addr, &g_dst_addr, g_dst_type,
                                  BT_SECURITY_LOW);

    if (fd < 0) {
        mainloop_add_timeout(2000, reconnect_cb, NULL, NULL);
        return false;
    }

    struct client* cli = client_create(fd, 0);
    if (!cli) {
        close(fd);
        mainloop_add_timeout(2000, reconnect_cb, NULL, NULL);
        return false;
    }

    pthread_mutex_lock(&g_state.lock);
    g_state.connected = true;
    pthread_mutex_unlock(&g_state.lock);

    return true;
}

void ble_client_stop(void) {
    mainloop_quit();
}

bool ble_get_temperature(float* out) {
    bool ok;

    pthread_mutex_lock(&g_state.lock);
    ok = g_state.has_temp;
    if (ok)
        *out = g_state.temperature;
    pthread_mutex_unlock(&g_state.lock);

    return ok;
}

bool ble_get_pressure(float* out) {
    bool ok;

    pthread_mutex_lock(&g_state.lock);
    ok = g_state.has_press;
    if (ok)
        *out = g_state.pressure;
    pthread_mutex_unlock(&g_state.lock);

    return ok;
}

bool ble_get_humidity(float* out) {
    bool ok;

    pthread_mutex_lock(&g_state.lock);
    ok = g_state.has_humid;
    if (ok)
        *out = g_state.humidity;
    pthread_mutex_unlock(&g_state.lock);

    return ok;
}

bool ble_is_connected(void) {
    bool connected;

    pthread_mutex_lock(&g_state.lock);
    connected = g_state.connected;
    pthread_mutex_unlock(&g_state.lock);

    return connected;
}
