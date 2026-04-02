#include "BLE_commands.h"
#include <errno.h>
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

/* Keep BLE sending out of the haptic loop: enqueue short text payloads and
 * transmit from a dedicated thread. */
#define BLE_MSG_MAX_LEN      20
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
    ble_queue_text(cmd);
}
