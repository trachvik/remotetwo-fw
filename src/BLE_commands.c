#include "BLE_commands.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ble_commands, LOG_LEVEL_DBG);

/* Keep BLE sending out of the haptic loop: enqueue text payloads and
 * transmit from a dedicated thread. */
#define BLE_MSG_MAX_LEN      64
#define BLE_TX_QUEUE_LEN      16
#define BLE_TX_STACK_SIZE   1024
#define BLE_TX_THREAD_PRIO    7

struct ble_tx_msg {
    uint8_t len;
    char payload[BLE_MSG_MAX_LEN];
};

K_MSGQ_DEFINE(ble_tx_msgq, sizeof(struct ble_tx_msg), BLE_TX_QUEUE_LEN, 4);
K_SEM_DEFINE(ble_ready_sem, 0, 1);
K_THREAD_STACK_DEFINE(ble_tx_stack, BLE_TX_STACK_SIZE);
static struct k_thread ble_tx_thread_data;

static bool g_ble_connected;
static struct bt_conn *g_current_conn;
static bool g_nus_notif_enabled;
static uint16_t g_tx_cmd_id;

static struct ble_printer_state g_state;
static struct ble_last_ack g_last_ack;
static K_MUTEX_DEFINE(g_proto_lock);

static bool str_to_bool(const char *s, bool *out)
{
    if (s == NULL || out == NULL) {
        return false;
    }

    if ((strcmp(s, "1") == 0) || (strcmp(s, "true") == 0) || (strcmp(s, "on") == 0)) {
        *out = true;
        return true;
    }
    if ((strcmp(s, "0") == 0) || (strcmp(s, "false") == 0) || (strcmp(s, "off") == 0)) {
        *out = false;
        return true;
    }

    return false;
}

static bool str_to_float(const char *s, float *out)
{
    if (s == NULL || out == NULL) {
        return false;
    }

    char *endp = NULL;
    float val = strtof(s, &endp);
    if (endp == s || (endp != NULL && *endp != '\0')) {
        return false;
    }

    *out = val;
    return true;
}

static void parse_ack_message(char *msg)
{
    char *saveptr = NULL;
    char *tok = strtok_r(msg, ":", &saveptr); /* ack */
    ARG_UNUSED(tok);

    char *id_tok = strtok_r(NULL, ":", &saveptr);
    char *status_tok = strtok_r(NULL, ":", &saveptr);
    char *reason_tok = strtok_r(NULL, ":", &saveptr);

    if (id_tok == NULL || status_tok == NULL) {
        LOG_WRN("Malformed ACK");
        return;
    }

    long id_val = strtol(id_tok, NULL, 10);
    bool ok = (strcmp(status_tok, "ok") == 0);

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    g_last_ack.valid = true;
    g_last_ack.id = (uint16_t)id_val;
    g_last_ack.ok = ok;
    if (ok || reason_tok == NULL) {
        g_last_ack.reason[0] = '\0';
    } else {
        (void)snprintf(g_last_ack.reason, sizeof(g_last_ack.reason), "%s", reason_tok);
    }
    k_mutex_unlock(&g_proto_lock);

    if (ok) {
        LOG_INF("ACK id=%ld ok", id_val);
    } else {
        LOG_WRN("ACK id=%ld deny=%s", id_val, (reason_tok != NULL) ? reason_tok : "unknown");
    }
}

static void parse_state_pos(char *saveptr)
{
    float x = g_state.pos_x;
    float y = g_state.pos_y;
    float z = g_state.pos_z;
    float e = g_state.pos_e;

    while (1) {
        char *axis = strtok_r(NULL, ":", &saveptr);
        char *val = strtok_r(NULL, ":", &saveptr);
        if (axis == NULL || val == NULL) {
            break;
        }

        float parsed = 0.0f;
        if (!str_to_float(val, &parsed)) {
            continue;
        }

        if (strcmp(axis, "x") == 0) {
            x = parsed;
        } else if (strcmp(axis, "y") == 0) {
            y = parsed;
        } else if (strcmp(axis, "z") == 0) {
            z = parsed;
        } else if (strcmp(axis, "e") == 0) {
            e = parsed;
        }
    }

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    g_state.valid = true;
    g_state.pos_x = x;
    g_state.pos_y = y;
    g_state.pos_z = z;
    g_state.pos_e = e;
    k_mutex_unlock(&g_proto_lock);
}

static void parse_state_temp(char *saveptr)
{
    float te = g_state.temp_e;
    float tb = g_state.temp_b;

    while (1) {
        char *name = strtok_r(NULL, ":", &saveptr);
        char *val = strtok_r(NULL, ":", &saveptr);
        if (name == NULL || val == NULL) {
            break;
        }

        float parsed = 0.0f;
        if (!str_to_float(val, &parsed)) {
            continue;
        }

        if (strcmp(name, "e") == 0) {
            te = parsed;
        } else if (strcmp(name, "b") == 0) {
            tb = parsed;
        }
    }

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    g_state.valid = true;
    g_state.temp_e = te;
    g_state.temp_b = tb;
    k_mutex_unlock(&g_proto_lock);
}

static void parse_state_homed(char *saveptr)
{
    bool hx = g_state.homed_x;
    bool hy = g_state.homed_y;
    bool hz = g_state.homed_z;

    while (1) {
        char *axis = strtok_r(NULL, ":", &saveptr);
        char *val = strtok_r(NULL, ":", &saveptr);
        if (axis == NULL || val == NULL) {
            break;
        }

        bool parsed = false;
        if (!str_to_bool(val, &parsed)) {
            continue;
        }

        if (strcmp(axis, "x") == 0) {
            hx = parsed;
        } else if (strcmp(axis, "y") == 0) {
            hy = parsed;
        } else if (strcmp(axis, "z") == 0) {
            hz = parsed;
        }
    }

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    g_state.valid = true;
    g_state.homed_x = hx;
    g_state.homed_y = hy;
    g_state.homed_z = hz;
    k_mutex_unlock(&g_proto_lock);
}

static void parse_state_message(char *msg)
{
    char *saveptr = NULL;
    char *tok = strtok_r(msg, ":", &saveptr); /* state */
    ARG_UNUSED(tok);

    char *subtype = strtok_r(NULL, ":", &saveptr);
    if (subtype == NULL) {
        LOG_WRN("Malformed STATE");
        return;
    }

    if (strcmp(subtype, "pos") == 0) {
        parse_state_pos(saveptr);
    } else if (strcmp(subtype, "temp") == 0) {
        parse_state_temp(saveptr);
    } else if (strcmp(subtype, "homed") == 0) {
        parse_state_homed(saveptr);
    } else if (strcmp(subtype, "can_move") == 0) {
        char *val = strtok_r(NULL, ":", &saveptr);
        bool parsed = false;
        if (str_to_bool(val, &parsed)) {
            k_mutex_lock(&g_proto_lock, K_FOREVER);
            g_state.valid = true;
            g_state.can_move = parsed;
            k_mutex_unlock(&g_proto_lock);
        }
    } else if (strcmp(subtype, "printing") == 0) {
        char *val = strtok_r(NULL, ":", &saveptr);
        bool parsed = false;
        if (str_to_bool(val, &parsed)) {
            k_mutex_lock(&g_proto_lock, K_FOREVER);
            g_state.valid = true;
            g_state.printing = parsed;
            k_mutex_unlock(&g_proto_lock);
        }
    } else if (strcmp(subtype, "tool") == 0) {
        char *val = strtok_r(NULL, ":", &saveptr);
        if (val != NULL) {
            long tool = strtol(val, NULL, 10);
            k_mutex_lock(&g_proto_lock, K_FOREVER);
            g_state.valid = true;
            g_state.tool = (int)tool;
            k_mutex_unlock(&g_proto_lock);
        }
    }
}

static void nus_notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);
    g_nus_notif_enabled = enabled;
    LOG_INF("NUS TX notifications %s", enabled ? "enabled" : "disabled");
}

static void nus_received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);

    char tmp[BLE_MSG_MAX_LEN];
    size_t copy_len = len;
    if (copy_len >= sizeof(tmp)) {
        copy_len = sizeof(tmp) - 1;
    }
    memcpy(tmp, data, copy_len);
    tmp[copy_len] = '\0';

    LOG_INF("NUS RX: %s", tmp);

    if (strncmp(tmp, "ack:", 4) == 0) {
        parse_ack_message(tmp);
        return;
    }
    if (strncmp(tmp, "state:", 6) == 0 || strncmp(tmp, "status:", 7) == 0) {
        parse_state_message(tmp);
        return;
    }
}

static struct bt_nus_cb nus_cb = {
    .notif_enabled = nus_notif_enabled,
    .received = nus_received,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0) {
        LOG_WRN("BLE connect failed (err %u)", err);
        return;
    }

    if (g_current_conn != NULL) {
        bt_conn_unref(g_current_conn);
    }
    g_current_conn = bt_conn_ref(conn);
    g_ble_connected = true;
    g_nus_notif_enabled = false;
    LOG_INF("BLE connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (g_current_conn != NULL) {
        bt_conn_unref(g_current_conn);
        g_current_conn = NULL;
    }
    g_ble_connected = false;
    g_nus_notif_enabled = false;
    LOG_INF("BLE disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void ble_tx_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    k_sem_take(&ble_ready_sem, K_FOREVER);

    while (1) {
        struct ble_tx_msg msg;
        int qret = k_msgq_get(&ble_tx_msgq, &msg, K_FOREVER);
        if (qret != 0) {
            continue;
        }

        if (!g_ble_connected || !g_nus_notif_enabled || g_current_conn == NULL) {
            continue;
        }

        struct bt_conn *conn = bt_conn_ref(g_current_conn);
        int ret = bt_nus_send(conn, msg.payload, msg.len);
        if (ret != 0) {
            LOG_WRN("bt_nus_send failed (%d)", ret);
        }

        bt_conn_unref(conn);
    }
}

int ble_commands_init(void)
{
    LOG_INF("Initializing BLE stack");

    int err = bt_enable(NULL);
    if (err != 0) {
        LOG_ERR("bt_enable failed (%d)", err);
        return err;
    }

    err = bt_nus_cb_register(&nus_cb, NULL);
    if (err != 0) {
        LOG_ERR("bt_nus_cb_register failed (%d)", err);
        return err;
    }

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    const struct bt_data sd[] = {
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
    };

    static const struct bt_le_adv_param adv_param = {
        .id = 0,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_CONN,
        .interval_min = 0x0030,
        .interval_max = 0x0060,
        .peer = NULL,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err != 0) {
        LOG_ERR("Advertising start failed (%d)", err);
        return err;
    }

    LOG_INF("BLE NUS advertising as '%s'", CONFIG_BT_DEVICE_NAME);

    k_thread_create(&ble_tx_thread_data, ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ble_tx_stack),
                    ble_tx_thread_fn, NULL, NULL, NULL,
                    BLE_TX_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&ble_tx_thread_data, "ble_tx");
    k_sem_give(&ble_ready_sem);

    return 0;
}

static void ble_queue_text(const char *text)
{
    if (!g_ble_connected || !g_nus_notif_enabled) {
        return;
    }

    struct ble_tx_msg msg;
    size_t len = strlen(text);

    if (len >= BLE_MSG_MAX_LEN) {
        len = BLE_MSG_MAX_LEN - 1;
    }

    memcpy(msg.payload, text, len);
    msg.payload[len] = '\0';
    msg.len = (uint8_t)len;

    int ret = k_msgq_put(&ble_tx_msgq, &msg, K_NO_WAIT);
    if (ret == -ENOMSG) {
        LOG_WRN("BLE TX queue full, dropping '%s'", msg.payload);
    }
}

void ble_send_gcode(const char *cmd)
{
    if (cmd == NULL) {
        return;
    }

    uint16_t id = ++g_tx_cmd_id;
    if (id == 0U) {
        id = ++g_tx_cmd_id;
    }

    char framed[BLE_MSG_MAX_LEN];
    int ret = snprintf(framed, sizeof(framed), "cmd:%u:%s", id, cmd);
    if (ret <= 0 || ret >= (int)sizeof(framed)) {
        LOG_WRN("Command too long, dropping '%s'", cmd);
        return;
    }

    ble_queue_text(framed);
}

bool ble_printer_state_get(struct ble_printer_state *out_state)
{
    if (out_state == NULL) {
        return false;
    }

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    *out_state = g_state;
    k_mutex_unlock(&g_proto_lock);

    return out_state->valid;
}

bool ble_printer_can_move(void)
{
    bool can_move = true;
    k_mutex_lock(&g_proto_lock, K_FOREVER);
    if (g_state.valid) {
        can_move = g_state.can_move;
    }
    k_mutex_unlock(&g_proto_lock);
    return can_move;
}

bool ble_last_ack_get(struct ble_last_ack *out_ack)
{
    if (out_ack == NULL) {
        return false;
    }

    k_mutex_lock(&g_proto_lock, K_FOREVER);
    *out_ack = g_last_ack;
    k_mutex_unlock(&g_proto_lock);

    return out_ack->valid;
}
