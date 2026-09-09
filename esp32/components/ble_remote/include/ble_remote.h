#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dispenser.h"
#include "esp_err.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ble_remote_t *ble_remote_handle_t;

    /* Opcodes accepted by the command characteristic, first byte of the write */
    typedef enum
    {
        BLE_REMOTE_COMMAND_HOME    = 0x01, /* park the drum on the home magnet */
        BLE_REMOTE_COMMAND_ADVANCE = 0x02, /* turn on to the next magnet, dispensing a treat */
        BLE_REMOTE_COMMAND_RETREAT = 0x03, /* turn back to the previous magnet, dispensing nothing */

        /*
         * 0x10 plus a slot number parks that slot at the opening, so 0x10 to
         * 0x13 cover a four-slot drum. Slot 0 is home, making 0x10 another way
         * to write BLE_REMOTE_COMMAND_HOME. Needs a calibrated drum: without a
         * slot map the firmware answers BLE_REMOTE_RESULT_NO_MAP.
         */
        BLE_REMOTE_COMMAND_SLOT_BASE = 0x10,
    } ble_remote_command_t;

/*
 * Fifth byte of the status characteristic when the drum is not sitting on a slot
 * that can be named: parked between magnets, still turning, or on a drum with no
 * slot map at all.
 */
#define BLE_REMOTE_SLOT_NONE 0xff

    /* First byte of the status characteristic */
    typedef enum
    {
        BLE_REMOTE_STATE_IDLE = 0x00,
        BLE_REMOTE_STATE_BUSY = 0x01,
    } ble_remote_state_t;

    /* Flags byte of the time characteristic */
    typedef enum
    {
        BLE_REMOTE_CLOCK_PRESENT = 0x01, /* the board has an RTC and it answered */
        BLE_REMOTE_CLOCK_VALID   = 0x02, /* its time has been set since it last lost power */
    } ble_remote_clock_flag_t;

    /* Third byte of the status characteristic: how the last command ended */
    typedef enum
    {
        BLE_REMOTE_RESULT_NONE      = 0x00, /* nothing has been asked for yet */
        BLE_REMOTE_RESULT_OK        = 0x01,
        BLE_REMOTE_RESULT_TIMEOUT   = 0x02, /* no magnet arrived within the budget */
        BLE_REMOTE_RESULT_NOT_FOUND = 0x03, /* a full drum went by without the slot asked for */
        BLE_REMOTE_RESULT_FAILED    = 0x04, /* anything else the motion layer reported */
        BLE_REMOTE_RESULT_NO_MAP    = 0x05, /* the drum has no slot map, so slots cannot be told apart */
    } ble_remote_result_t;

    typedef struct
    {
        dispenser_handle_t drum_handle; /* initialized dispenser motion handle */

        /*
         * Both are optional and may be NULL, the board running without an RTC
         * or without a schedule being something main() already tolerates. The
         * characteristics then read back as "no clock" and "no schedule" rather
         * than failing, so the app can say so.
         */
        rv3028_handle_t    rtc_handle;      /* RTC read by the time characteristic */
        scheduler_handle_t schedule_handle; /* schedule read by the schedule characteristic */

        const char *device_name;       /* advertised name, NULL selects "Treat Dispenser" */
        uint32_t    pairing_window_ms; /* how long new phones may pair after start, 0 selects 60000 ms */

        /*
         * Chime played when a move came in over this link, in place of the one
         * the dispenser is configured with. The phone shows the move finishing
         * on screen, so a full melody every time is noise; a repeat_count of 0
         * keeps the remote silent altogether.
         */
        dispenser_chime_t remote_chime;

        uint32_t task_stack_size; /* command worker stack, 0 selects 4096 */
        uint32_t task_priority;   /* command worker priority, 0 selects 5 */
    } ble_remote_config_t;

    typedef struct
    {
        bool     pairing_open;    /* new phones are allowed to pair right now */
        uint32_t pairing_left_ms; /* time left in the pairing window, 0 when it is shut */
        bool     connected;       /* a phone is connected */
        bool     encrypted;       /* that link is encrypted, so the characteristics are reachable */
        bool     busy;            /* a command is being carried out */
        uint32_t bond_count;      /* phones currently bonded */
    } ble_remote_status_t;

    /**
     * Bring up the BLE peripheral and start advertising.
     *
     * Starts the NimBLE controller and host, publishes the dispenser service and
     * opens the pairing window for the configured time. Only one instance can
     * exist because the NimBLE callbacks are global.
     *
     * Every characteristic needs an encrypted link, so a phone that has not
     * bonded can discover the service but cannot drive the drum. Pairing uses
     * LE Secure Connections with Just Works, the board having no display or
     * keypad; the pairing window is what keeps that from being an open door.
     *
     * @param config Peripheral configuration.
     * @param out_handle Receives the handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle,
     *         ESP_ERR_INVALID_STATE when a peripheral is already running,
     *         ESP_ERR_NO_MEM when the handle or its task cannot be allocated, or
     *         an error from the NimBLE port.
     */
    esp_err_t ble_remote_start(const ble_remote_config_t *config, ble_remote_handle_t *out_handle);

    /**
     * Read what the link is doing.
     *
     * @param handle Peripheral handle.
     * @param out_status Receives the current state.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing argument.
     */
    esp_err_t ble_remote_get_status(ble_remote_handle_t handle, ble_remote_status_t *out_status);

    /**
     * Re-open the pairing window for the configured time.
     *
     * The window normally only opens at start-up. This is the way to let another
     * phone in later without power cycling the board.
     *
     * @param handle Peripheral handle.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle, or an
     *         error from the timer.
     */
    esp_err_t ble_remote_open_pairing(ble_remote_handle_t handle);

    /**
     * Shut the pairing window now, before it would have expired.
     *
     * @param handle Peripheral handle.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle.
     */
    esp_err_t ble_remote_close_pairing(ble_remote_handle_t handle);

    /**
     * Delete every bond and drop the current link.
     *
     * The phone keeps its own half of the bond, so it has to forget the
     * dispenser in the Android Bluetooth settings before it can pair again.
     *
     * @param handle Peripheral handle.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle, or an
     *         error from the NimBLE store.
     */
    esp_err_t ble_remote_forget_bonds(ble_remote_handle_t handle);

#ifdef __cplusplus
}
#endif
