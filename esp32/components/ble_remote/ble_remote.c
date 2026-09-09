#include "ble_remote.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Declared by the NimBLE config store, which has no public header of its own */
void ble_store_config_init(void);

#define BLE_REMOTE_DEFAULT_STACK_SIZE 4096
#define BLE_REMOTE_DEFAULT_PRIORITY   5
#define BLE_REMOTE_DEFAULT_NAME       "Treat Dispenser"

/*
 * Sixty seconds is long enough to walk to the dispenser, open the app and let
 * Android work through discovery and pairing, and short enough that an
 * unattended board is not sitting there waiting to be adopted.
 */
#define BLE_REMOTE_DEFAULT_PAIRING_MS 60000

/* A treat is a treat; there is no point queueing more than a couple of presses */
#define BLE_REMOTE_QUEUE_LENGTH 4

/* Sized from the bond store itself, so the two cannot drift apart */
#define BLE_REMOTE_MAX_BONDS CONFIG_BT_NIMBLE_MAX_BONDS

/*
 * As many slots as the schedule characteristic will report. Three is what the
 * board ships with; the cap is only here so the payload stays inside one ATT
 * read no matter what the schedule grows to.
 */
#define BLE_REMOTE_MAX_SLOTS 12

/* Header of the schedule payload: slot count, then the hour and minute the alarm is armed for */
#define BLE_REMOTE_SCHEDULE_HEADER 3

/* Stands in for the next hour and minute when there is no schedule to be armed for */
#define BLE_REMOTE_SLOT_NONE 0xff

/*
 * One random base UUID with the last byte identifying the service and its
 * characteristics, so the whole family is recognizable in a scanner:
 *
 *   service ffc50e4e-afd5-4edd-86a1-b41c94120001
 *   command ffc50e4e-afd5-4edd-86a1-b41c94120002
 *   status  ffc50e4e-afd5-4edd-86a1-b41c94120003
 */
#define BLE_REMOTE_UUID128(last_byte)                                                                               \
    BLE_UUID128_INIT(last_byte, 0x00, 0x12, 0x94, 0x1c, 0xb4, 0xa1, 0x86, 0xdd, 0x4e, 0xd5, 0xaf, 0x4e, 0x0e, 0xc5, \
                     0xff)

/* Layout of the status characteristic, read and notified */
typedef struct __attribute__((packed))
{
    uint8_t state;        /* ble_remote_state_t */
    uint8_t last_command; /* ble_remote_command_t of the last command, 0 for none */
    uint8_t last_result;  /* ble_remote_result_t */
    uint8_t at_home;      /* drum is parked on the home magnet, only meaningful while idle */

    /*
     * Slot at the opening, or BLE_REMOTE_SLOT_NONE. Appended after at_home
     * rather than replacing it so that a phone built against the four-byte
     * layout keeps working: it reads the bytes it knows and ignores this one.
     */
    uint8_t slot;
} ble_remote_payload_t;

/* Wall clock as the RV-3028 keeps it; there is no timezone, it is whatever was set */
typedef struct __attribute__((packed))
{
    uint8_t  flags; /* BLE_REMOTE_CLOCK_* */
    uint16_t year;  /* full year, e.g. 2026 */
    uint8_t  month; /* 1-12 */
    uint8_t  day;   /* 1-31 */
    uint8_t  hour;  /* 0-23 */
    uint8_t  minute;
    uint8_t  second;
} ble_remote_time_payload_t;

typedef struct ble_remote_t
{
    dispenser_handle_t drum;
    dispenser_chime_t  remote_chime;
    rv3028_handle_t    rtc;
    scheduler_handle_t schedule;

    char     device_name[32];
    uint32_t pairing_window_ms;

    QueueHandle_t     commands;
    TaskHandle_t      task;
    SemaphoreHandle_t lock;

    esp_timer_handle_t pairing_timer;
    bool               pairing_open;
    int64_t            pairing_closes_us;

    uint8_t  own_addr_type;
    uint16_t conn_handle;
    bool     connected;
    bool     encrypted;
    bool     notify_enabled;

    uint8_t state;
    uint8_t last_command;
    uint8_t last_result;
} ble_remote_ctx_t;

static const char *TAG = "ble_remote";

/* NimBLE hands its callbacks no argument of ours, so the one instance has to be reachable statically */
static ble_remote_ctx_t *s_ctx;
static uint16_t          s_status_val_handle;

static const ble_uuid128_t ble_remote_svc_uuid      = BLE_REMOTE_UUID128(0x01);
static const ble_uuid128_t ble_remote_command_uuid  = BLE_REMOTE_UUID128(0x02);
static const ble_uuid128_t ble_remote_status_uuid   = BLE_REMOTE_UUID128(0x03);
static const ble_uuid128_t ble_remote_time_uuid     = BLE_REMOTE_UUID128(0x04);
static const ble_uuid128_t ble_remote_schedule_uuid = BLE_REMOTE_UUID128(0x05);

static void ble_remote_host_task(void *arg);
static void ble_remote_worker_task(void *arg);
static void ble_remote_on_sync(void);
static void ble_remote_on_reset(int reason);
static void ble_remote_advertise(void);
static int  ble_remote_gap_event(struct ble_gap_event *event, void *arg);
static int  ble_remote_command_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                      void *arg);
static int  ble_remote_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                     void *arg);
static int  ble_remote_time_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                   void *arg);
static int  ble_remote_schedule_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                       void *arg);
static esp_err_t ble_remote_register_services(void);
static void      ble_remote_pairing_expired(void *arg);
static void      ble_remote_set_pairing(ble_remote_ctx_t *ctx, bool open);
static bool      ble_remote_peer_is_bonded(const ble_addr_t *peer_id_addr);
static uint32_t  ble_remote_count_bonds(void);
static void      ble_remote_build_payload(ble_remote_ctx_t *ctx, ble_remote_payload_t *out_payload);
static void      ble_remote_notify_status(ble_remote_ctx_t *ctx);
static uint8_t   ble_remote_map_result(esp_err_t err);
static bool      ble_remote_is_slot_command(uint8_t opcode);

static struct ble_gatt_chr_def ble_remote_characteristics[] = {
    {
        /* Write only: nothing to read back here, the status characteristic is the answer */
        .uuid      = &ble_remote_command_uuid.u,
        .access_cb = ble_remote_command_access,
        .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
    },
    {
        .uuid       = &ble_remote_status_uuid.u,
        .access_cb  = ble_remote_status_access,
        .val_handle = &s_status_val_handle,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        /*
         * Read on demand rather than notified: the clock and the schedule only
         * change when somebody sets them, which is not something that happens
         * over this link.
         */
        .uuid      = &ble_remote_time_uuid.u,
        .access_cb = ble_remote_time_access,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid      = &ble_remote_schedule_uuid.u,
        .access_cb = ble_remote_schedule_access,
        .flags     = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        0,
    },
};

static const struct ble_gatt_svc_def ble_remote_services[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &ble_remote_svc_uuid.u,
        .characteristics = ble_remote_characteristics,
    },
    {
        0,
    },
};

esp_err_t ble_remote_start(const ble_remote_config_t *config, ble_remote_handle_t *out_handle)
{
    esp_err_t         err;
    ble_remote_ctx_t *ctx;
    int               rc;

    if (!config || !config->drum_handle || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(ble_remote_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->drum              = config->drum_handle;
    ctx->remote_chime      = config->remote_chime;
    ctx->rtc               = config->rtc_handle;
    ctx->schedule          = config->schedule_handle;
    ctx->pairing_window_ms = config->pairing_window_ms ? config->pairing_window_ms : BLE_REMOTE_DEFAULT_PAIRING_MS;
    ctx->conn_handle       = BLE_HS_CONN_HANDLE_NONE;
    ctx->state             = BLE_REMOTE_STATE_IDLE;
    ctx->last_result       = BLE_REMOTE_RESULT_NONE;

    strlcpy(ctx->device_name, config->device_name ? config->device_name : BLE_REMOTE_DEFAULT_NAME,
            sizeof(ctx->device_name));

    ctx->commands = xQueueCreate(BLE_REMOTE_QUEUE_LENGTH, sizeof(uint8_t));
    ctx->lock     = xSemaphoreCreateMutex();
    if (!ctx->commands || !ctx->lock)
    {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    const esp_timer_create_args_t pairing_timer_args = {
        .callback = ble_remote_pairing_expired,
        .arg      = ctx,
        .name     = "ble_pairing",
    };

    err = esp_timer_create(&pairing_timer_args, &ctx->pairing_timer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create the pairing timer (%s)", esp_err_to_name(err));
        goto fail;
    }

    if (xTaskCreate(ble_remote_worker_task, "ble_remote",
                    config->task_stack_size ? config->task_stack_size : BLE_REMOTE_DEFAULT_STACK_SIZE, ctx,
                    config->task_priority ? (UBaseType_t) config->task_priority : BLE_REMOTE_DEFAULT_PRIORITY,
                    &ctx->task) != pdPASS)
    {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    /* From here on the NimBLE callbacks can fire, so the instance has to be visible first */
    s_ctx = ctx;

    err = nimble_port_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start the NimBLE port (%s)", esp_err_to_name(err));
        goto fail_started;
    }

    ble_hs_cfg.reset_cb        = ble_remote_on_reset;
    ble_hs_cfg.sync_cb         = ble_remote_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /*
     * No display and no keypad, so Just Works is the only association model on
     * offer. It is unauthenticated, which is why the characteristics ask for
     * encryption but not authentication, and why pairing is only accepted
     * inside the window rather than whenever a stranger walks past.
     */
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    err = ble_remote_register_services();
    if (err != ESP_OK)
    {
        goto fail_started;
    }

    rc = ble_svc_gap_device_name_set(ctx->device_name);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set the device name (rc=%d)", rc);
        err = ESP_FAIL;
        goto fail_started;
    }

    ble_store_config_init();

    ble_remote_set_pairing(ctx, true);

    nimble_port_freertos_init(ble_remote_host_task);

    ESP_LOGI(TAG, "Advertising as %s, %lu bond(s) stored", ctx->device_name, (unsigned long) ble_remote_count_bonds());

    *out_handle = ctx;
    return ESP_OK;

fail_started:
    s_ctx = NULL;

fail:
    if (ctx->task)
    {
        vTaskDelete(ctx->task);
    }

    if (ctx->pairing_timer)
    {
        esp_timer_delete(ctx->pairing_timer);
    }

    if (ctx->lock)
    {
        vSemaphoreDelete(ctx->lock);
    }

    if (ctx->commands)
    {
        vQueueDelete(ctx->commands);
    }

    free(ctx);
    return err;
}

esp_err_t ble_remote_get_status(ble_remote_handle_t handle, ble_remote_status_t *out_status)
{
    if (!handle || !out_status)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);

    int64_t left_us = handle->pairing_closes_us - esp_timer_get_time();

    out_status->pairing_open    = handle->pairing_open;
    out_status->pairing_left_ms = (handle->pairing_open && left_us > 0) ? (uint32_t) (left_us / 1000) : 0;
    out_status->connected       = handle->connected;
    out_status->encrypted       = handle->encrypted;
    out_status->busy            = handle->state == BLE_REMOTE_STATE_BUSY;

    xSemaphoreGive(handle->lock);

    out_status->bond_count = ble_remote_count_bonds();

    return ESP_OK;
}

esp_err_t ble_remote_open_pairing(ble_remote_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ble_remote_set_pairing(handle, true);

    return ESP_OK;
}

esp_err_t ble_remote_close_pairing(ble_remote_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ble_remote_set_pairing(handle, false);

    return ESP_OK;
}

esp_err_t ble_remote_forget_bonds(ble_remote_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int rc = ble_store_clear();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to clear the bond store (rc=%d)", rc);
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "All bonds deleted, the phone has to forget the dispenser too");

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    uint16_t conn_handle = handle->conn_handle;
    xSemaphoreGive(handle->lock);

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    return ESP_OK;
}

static esp_err_t ble_remote_register_services(void)
{
    int rc = ble_gatts_count_cfg(ble_remote_services);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to size the GATT table (rc=%d)", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(ble_remote_services);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to add the dispenser service (rc=%d)", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void ble_remote_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_remote_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset, reason %d", reason);
}

static void ble_remote_on_sync(void)
{
    ble_remote_ctx_t *ctx = s_ctx;
    int               rc;

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to ensure an identity address (rc=%d)", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &ctx->own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to pick an address type (rc=%d)", rc);
        return;
    }

    ble_remote_advertise();
}

/*
 * A 128-bit service UUID plus the flags fills most of the 31 byte advertising
 * payload, so the name goes in the scan response. Android merges the two into
 * one scan record, which is what the app filters and displays on.
 */
static void ble_remote_advertise(void)
{
    ble_remote_ctx_t         *ctx = s_ctx;
    struct ble_hs_adv_fields  adv_fields;
    struct ble_hs_adv_fields  rsp_fields;
    struct ble_gap_adv_params adv_params;
    int                       rc;

    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags                = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128             = (ble_uuid128_t *) &ble_remote_svc_uuid;
    adv_fields.num_uuids128         = 1;
    adv_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set the advertising payload (rc=%d)", rc);
        return;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name             = (const uint8_t *) ctx->device_name;
    rsp_fields.name_len         = strlen(ctx->device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set the scan response (rc=%d)", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ctx->own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_remote_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY)
    {
        ESP_LOGE(TAG, "Failed to start advertising (rc=%d)", rc);
    }
}

static int ble_remote_gap_event(struct ble_gap_event *event, void *arg)
{
    ble_remote_ctx_t        *ctx = s_ctx;
    struct ble_gap_conn_desc desc;
    bool                     pairing_open;

    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0)
            {
                ESP_LOGW(TAG, "Connection attempt failed (status=%d)", event->connect.status);
                ble_remote_advertise();
                break;
            }

            if (ble_gap_conn_find(event->connect.conn_handle, &desc) != 0)
            {
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                break;
            }

            xSemaphoreTake(ctx->lock, portMAX_DELAY);
            pairing_open = ctx->pairing_open;
            xSemaphoreGive(ctx->lock);

            /*
             * With the window shut only a phone we already hold keys for is
             * let in. An unbonded peer could not drive the drum anyway, every
             * characteristic asking for encryption, but dropping it here keeps
             * the single connection slot free for the phone that owns the
             * dispenser.
             */
            if (!pairing_open && !ble_remote_peer_is_bonded(&desc.peer_id_addr))
            {
                ESP_LOGW(TAG, "Rejecting an unbonded phone, the pairing window is shut");
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                break;
            }

            xSemaphoreTake(ctx->lock, portMAX_DELAY);
            ctx->conn_handle    = event->connect.conn_handle;
            ctx->connected      = true;
            ctx->encrypted      = desc.sec_state.encrypted;
            ctx->notify_enabled = false;
            xSemaphoreGive(ctx->lock);

            ESP_LOGI(TAG, "Connected, handle %u", (unsigned) event->connect.conn_handle);
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason %d", event->disconnect.reason);

            xSemaphoreTake(ctx->lock, portMAX_DELAY);
            ctx->conn_handle    = BLE_HS_CONN_HANDLE_NONE;
            ctx->connected      = false;
            ctx->encrypted      = false;
            ctx->notify_enabled = false;
            xSemaphoreGive(ctx->lock);

            ble_remote_advertise();
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0)
            {
                xSemaphoreTake(ctx->lock, portMAX_DELAY);
                ctx->encrypted = desc.sec_state.encrypted;
                xSemaphoreGive(ctx->lock);

                ESP_LOGI(TAG, "Link %s, bonded=%d", desc.sec_state.encrypted ? "encrypted" : "not encrypted",
                         desc.sec_state.bonded);
            }
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_status_val_handle)
            {
                xSemaphoreTake(ctx->lock, portMAX_DELAY);
                ctx->notify_enabled = event->subscribe.cur_notify;
                xSemaphoreGive(ctx->lock);

                ESP_LOGI(TAG, "Status notifications %s", event->subscribe.cur_notify ? "on" : "off");
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "ATT MTU is %u", (unsigned) event->mtu.value);
            break;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            /*
             * The phone has forgotten its half of the bond and wants to start
             * over. That is only allowed while the window is open; otherwise
             * the stale bond stays and the link goes down.
             *
             * Dropping the link matters: ignoring the request on its own
             * leaves the phone waiting on a pairing request that is never
             * answered, which reads as a hang rather than as a refusal. A
             * disconnect is a failure the phone can see and report.
             */
            xSemaphoreTake(ctx->lock, portMAX_DELAY);
            pairing_open = ctx->pairing_open;
            xSemaphoreGive(ctx->lock);

            if (!pairing_open)
            {
                ESP_LOGW(TAG, "Refusing a re-pairing request, the pairing window is shut");
                ble_gap_terminate(event->repeat_pairing.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return BLE_GAP_REPEAT_PAIRING_IGNORE;
            }

            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0)
            {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }

            return BLE_GAP_REPEAT_PAIRING_RETRY;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_remote_advertise();
            break;

        default:
            break;
    }

    return 0;
}

static int ble_remote_command_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                     void *arg)
{
    ble_remote_ctx_t *ctx = s_ctx;
    uint8_t           opcode;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (OS_MBUF_PKTLEN(ctxt->om) < 1)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (ble_hs_mbuf_to_flat(ctxt->om, &opcode, sizeof(opcode), NULL) != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (opcode != BLE_REMOTE_COMMAND_HOME && opcode != BLE_REMOTE_COMMAND_ADVANCE &&
        opcode != BLE_REMOTE_COMMAND_RETREAT && !ble_remote_is_slot_command(opcode))
    {
        ESP_LOGW(TAG, "Unknown command 0x%02x", opcode);
        return BLE_ATT_ERR_UNLIKELY;
    }

    /*
     * A move takes seconds and the host task has to stay responsive, so the
     * opcode is handed to the worker and the write is acknowledged straight
     * away. The app watches the status characteristic to see it finish.
     */
    if (xQueueSend(ctx->commands, &opcode, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Command queue full, dropping 0x%02x", opcode);
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}

static int ble_remote_status_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                    void *arg)
{
    ble_remote_payload_t payload;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    ble_remote_build_payload(s_ctx, &payload);

    if (os_mbuf_append(ctxt->om, &payload, sizeof(payload)) != 0)
    {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}

/*
 * The RV-3028 keeps plain wall clock time with no timezone attached, so the
 * fields go out exactly as it reports them and the phone shows them as they
 * are. Reporting the flags separately lets the app tell a board with no clock
 * apart from one whose clock has never been set.
 */
static int ble_remote_time_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                  void *arg)
{
    ble_remote_ctx_t         *ctx     = s_ctx;
    ble_remote_time_payload_t payload = {0};
    struct tm                 now     = {0};

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctx->rtc && rv3028_get_time(ctx->rtc, &now) == ESP_OK)
    {
        bool valid = false;

        payload.flags = BLE_REMOTE_CLOCK_PRESENT;

        if (rv3028_is_time_valid(ctx->rtc, &valid) == ESP_OK && valid)
        {
            payload.flags |= BLE_REMOTE_CLOCK_VALID;
        }

        payload.year   = (uint16_t) (now.tm_year + 1900);
        payload.month  = (uint8_t) (now.tm_mon + 1);
        payload.day    = (uint8_t) now.tm_mday;
        payload.hour   = (uint8_t) now.tm_hour;
        payload.minute = (uint8_t) now.tm_min;
        payload.second = (uint8_t) now.tm_sec;
    }

    if (os_mbuf_append(ctxt->om, &payload, sizeof(payload)) != 0)
    {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}

/*
 * Slot count, then the hour and minute the RTC alarm is armed for, then the
 * schedule itself as hour and minute pairs. A count of zero says there is no
 * schedule, which is what a board with no RTC reports.
 */
static int ble_remote_schedule_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt,
                                      void *arg)
{
    ble_remote_ctx_t       *ctx = s_ctx;
    uint8_t                 buf[BLE_REMOTE_SCHEDULE_HEADER + BLE_REMOTE_MAX_SLOTS * 2];
    size_t                  len   = BLE_REMOTE_SCHEDULE_HEADER;
    const scheduler_slot_t *slots = NULL;
    size_t                  count = 0;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    buf[0] = 0;
    buf[1] = BLE_REMOTE_SLOT_NONE;
    buf[2] = BLE_REMOTE_SLOT_NONE;

    if (ctx->schedule && scheduler_get_slots(ctx->schedule, &slots, &count) == ESP_OK)
    {
        scheduler_slot_t next = {0};

        if (count > BLE_REMOTE_MAX_SLOTS)
        {
            ESP_LOGW(TAG, "Reporting %d of %u slots, the rest do not fit", BLE_REMOTE_MAX_SLOTS, (unsigned) count);
            count = BLE_REMOTE_MAX_SLOTS;
        }

        buf[0] = (uint8_t) count;

        if (scheduler_get_next_slot(ctx->schedule, &next) == ESP_OK)
        {
            buf[1] = next.hour;
            buf[2] = next.minute;
        }

        for (size_t i = 0; i < count; i++)
        {
            buf[len++] = slots[i].hour;
            buf[len++] = slots[i].minute;
        }
    }

    if (os_mbuf_append(ctxt->om, buf, len) != 0)
    {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}

static void ble_remote_worker_task(void *arg)
{
    ble_remote_ctx_t *ctx = arg;
    uint8_t           opcode;

    for (;;)
    {
        if (xQueueReceive(ctx->commands, &opcode, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        xSemaphoreTake(ctx->lock, portMAX_DELAY);
        ctx->state        = BLE_REMOTE_STATE_BUSY;
        ctx->last_command = opcode;
        xSemaphoreGive(ctx->lock);

        ble_remote_notify_status(ctx);

        dispenser_result_t result;
        esp_err_t          err;
        dispenser_move_t   move;

        if (ble_remote_is_slot_command(opcode))
        {
            err = dispenser_go_to_slot(ctx->drum, opcode - BLE_REMOTE_COMMAND_SLOT_BASE, &ctx->remote_chime, &result);
        }
        else
        {
            switch (opcode)
            {
                case BLE_REMOTE_COMMAND_HOME:
                    move = DISPENSER_MOVE_HOME;
                    break;

                case BLE_REMOTE_COMMAND_RETREAT:
                    move = DISPENSER_MOVE_RETREAT;
                    break;

                default:
                    move = DISPENSER_MOVE_ADVANCE;
                    break;
            }

            err = dispenser_move(ctx->drum, move, &ctx->remote_chime, &result);
        }

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Command 0x%02x done in %lu ms", opcode, (unsigned long) result.elapsed_ms);
        }
        else
        {
            ESP_LOGE(TAG, "Command 0x%02x failed (%s)", opcode, esp_err_to_name(err));
        }

        xSemaphoreTake(ctx->lock, portMAX_DELAY);
        ctx->state       = BLE_REMOTE_STATE_IDLE;
        ctx->last_result = ble_remote_map_result(err);
        xSemaphoreGive(ctx->lock);

        ble_remote_notify_status(ctx);
    }
}

static void ble_remote_build_payload(ble_remote_ctx_t *ctx, ble_remote_payload_t *out_payload)
{
    bool at_home = false;
    int  slot    = DISPENSER_SLOT_NONE;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    out_payload->state        = ctx->state;
    out_payload->last_command = ctx->last_command;
    out_payload->last_result  = ctx->last_result;
    bool busy                 = ctx->state == BLE_REMOTE_STATE_BUSY;
    xSemaphoreGive(ctx->lock);

    /* The Hall reading only means anything once the drum has stopped moving */
    if (!busy)
    {
        dispenser_is_home(ctx->drum, &at_home);

        /*
         * Fails on a drum with no slot map, which is not worth reporting as an
         * error: the slot simply cannot be named, same as parking between two
         * magnets, and the app says so either way.
         */
        if (dispenser_get_slot(ctx->drum, &slot) != ESP_OK)
        {
            slot = DISPENSER_SLOT_NONE;
        }
    }

    out_payload->at_home = at_home ? 1 : 0;
    out_payload->slot    = slot == DISPENSER_SLOT_NONE ? BLE_REMOTE_SLOT_NONE : (uint8_t) slot;
}

static void ble_remote_notify_status(ble_remote_ctx_t *ctx)
{
    ble_remote_payload_t payload;
    uint16_t             conn_handle;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    conn_handle = (ctx->connected && ctx->notify_enabled) ? ctx->conn_handle : BLE_HS_CONN_HANDLE_NONE;
    xSemaphoreGive(ctx->lock);

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
    {
        return;
    }

    ble_remote_build_payload(ctx, &payload);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&payload, sizeof(payload));
    if (!om)
    {
        ESP_LOGW(TAG, "Out of mbufs, skipping a status notification");
        return;
    }

    int rc = ble_gatts_notify_custom(conn_handle, s_status_val_handle, om);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to notify the status (rc=%d)", rc);
    }
}

static void ble_remote_pairing_expired(void *arg)
{
    ble_remote_set_pairing(arg, false);
}

static void ble_remote_set_pairing(ble_remote_ctx_t *ctx, bool open)
{
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    bool was_open = ctx->pairing_open;

    ctx->pairing_open      = open;
    ctx->pairing_closes_us = open ? esp_timer_get_time() + (int64_t) ctx->pairing_window_ms * 1000 : 0;
    xSemaphoreGive(ctx->lock);

    /*
     * Keys are only handed out while the window is open. A phone that is
     * already bonded reconnects on the stored key and never pairs again, so
     * this costs it nothing.
     */
    ble_hs_cfg.sm_bonding = open ? 1 : 0;

    esp_timer_stop(ctx->pairing_timer);

    if (open)
    {
        esp_err_t err = esp_timer_start_once(ctx->pairing_timer, (uint64_t) ctx->pairing_window_ms * 1000);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to arm the pairing timer (%s)", esp_err_to_name(err));
        }

        ESP_LOGW(TAG, "Pairing window open for %lu s", (unsigned long) (ctx->pairing_window_ms / 1000));
    }
    else if (was_open)
    {
        ESP_LOGI(TAG, "Pairing window shut, %lu bond(s) stored", (unsigned long) ble_remote_count_bonds());
    }
}

/*
 * A bonded phone shows up here under its identity address, the controller
 * having resolved the private address it advertises with from the IRK stored
 * at pairing time. A phone we hold no IRK for cannot be resolved, which is
 * exactly the answer wanted: it is not bonded.
 */
static bool ble_remote_peer_is_bonded(const ble_addr_t *peer_id_addr)
{
    ble_addr_t peers[BLE_REMOTE_MAX_BONDS];
    int        count = 0;

    if (ble_store_util_bonded_peers(peers, &count, BLE_REMOTE_MAX_BONDS) != 0)
    {
        return false;
    }

    for (int i = 0; i < count; i++)
    {
        if (ble_addr_cmp(&peers[i], peer_id_addr) == 0)
        {
            return true;
        }
    }

    return false;
}

static uint32_t ble_remote_count_bonds(void)
{
    int count = 0;

    if (ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count) != 0)
    {
        return 0;
    }

    return (uint32_t) count;
}

/* Whether an opcode is one of the 0x10 + slot commands this drum has slots for */
static bool ble_remote_is_slot_command(uint8_t opcode)
{
    return opcode >= BLE_REMOTE_COMMAND_SLOT_BASE && opcode < BLE_REMOTE_COMMAND_SLOT_BASE + DISPENSER_SLOT_COUNT;
}

static uint8_t ble_remote_map_result(esp_err_t err)
{
    switch (err)
    {
        case ESP_OK:
            return BLE_REMOTE_RESULT_OK;

        case ESP_ERR_INVALID_STATE:
            return BLE_REMOTE_RESULT_NO_MAP;

        case ESP_ERR_TIMEOUT:
            return BLE_REMOTE_RESULT_TIMEOUT;

        case ESP_ERR_NOT_FOUND:
            return BLE_REMOTE_RESULT_NOT_FOUND;

        default:
            return BLE_REMOTE_RESULT_FAILED;
    }
}
