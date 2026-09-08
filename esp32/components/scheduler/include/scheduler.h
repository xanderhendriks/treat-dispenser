#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dispenser.h"
#include "esp_err.h"
#include "rv3028.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct scheduler_t *scheduler_handle_t;

    typedef struct
    {
        uint8_t hour;   /* 0-23 */
        uint8_t minute; /* 0-59 */
    } scheduler_slot_t;

    typedef struct
    {
        rv3028_handle_t    rtc_handle;  /* initialized RV-3028 handle */
        dispenser_handle_t drum_handle; /* initialized dispenser handle */

        int int_gpio_num; /* GPIO wired to the RTC INT pin */

        const scheduler_slot_t *slots;      /* dispense times, in ascending order */
        size_t                  slot_count; /* number of entries in slots */

        uint32_t task_stack_size; /* 0 selects 4096 bytes */
        int      task_priority;   /* 0 selects priority 5 */
    } scheduler_config_t;

    /**
     * Start the dispensing schedule.
     *
     * Arms the RV-3028 daily alarm for the next slot and starts a task that
     * waits on the RTC interrupt. Every time the alarm fires the task advances
     * the drum by one position and re-arms the alarm for the slot after that.
     *
     * The RTC INT pin is open drain with a 4k7 pull-up on the board, so the GPIO
     * is configured as a plain input and triggers on the falling edge.
     *
     * @param config Scheduler configuration.
     * @param out_handle Receives the handle on success.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a missing handle or an
     *         empty or out of range slot list, ESP_ERR_NO_MEM when the task or
     *         handle cannot be allocated, or an error from the GPIO or RTC
     *         driver.
     */
    esp_err_t scheduler_start(const scheduler_config_t *config, scheduler_handle_t *out_handle);

    /**
     * Re-arm the alarm for the next slot after the current RTC time.
     *
     * Call this after changing the RTC time, otherwise the armed alarm still
     * refers to the schedule as it looked under the old time.
     *
     * @param handle Scheduler handle.
     * @return ESP_OK on success or an error from the RTC driver.
     */
    esp_err_t scheduler_reschedule(scheduler_handle_t handle);

    /**
     * Get the slot the alarm is currently armed for.
     *
     * @param handle Scheduler handle.
     * @param out_slot Receives the slot.
     * @return ESP_OK on success.
     */
    esp_err_t scheduler_get_next_slot(scheduler_handle_t handle, scheduler_slot_t *out_slot);

    /**
     * Get the configured schedule.
     *
     * The returned pointer stays valid for as long as the handle does.
     *
     * @param handle Scheduler handle.
     * @param out_slots Receives a pointer to the slot list, may be NULL.
     * @param out_slot_count Receives the number of slots, may be NULL.
     * @return ESP_OK on success.
     */
    esp_err_t scheduler_get_slots(scheduler_handle_t handle, const scheduler_slot_t **out_slots,
                                  size_t *out_slot_count);

#ifdef __cplusplus
}
#endif
