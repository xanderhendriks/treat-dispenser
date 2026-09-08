#include "scheduler.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SCHEDULER_DEFAULT_STACK_SIZE 4096
#define SCHEDULER_DEFAULT_PRIORITY   5

#define SCHEDULER_MAX_SLOTS 24

/*
 * The RTC interrupt is what wakes the task up, but the alarm flag is also
 * re-read on this period. That covers the one case an edge cannot: a flag that
 * was already set, and INT therefore already low, before the GPIO interrupt was
 * hooked up, which leaves no edge for the task to see.
 */
#define SCHEDULER_RECHECK_MS 60000

typedef struct scheduler_t
{
    rv3028_handle_t    rtc;
    dispenser_handle_t drum;

    int int_gpio_num;

    scheduler_slot_t *slots;
    size_t            slot_count;
    scheduler_slot_t  next_slot;

    SemaphoreHandle_t alarm_signal;
    SemaphoreHandle_t lock;
    TaskHandle_t      task;
} scheduler_ctx_t;

static const char *TAG = "scheduler";

static void      scheduler_task(void *arg);
static void      scheduler_isr(void *arg);
static esp_err_t scheduler_arm_next(scheduler_ctx_t *ctx);
static esp_err_t scheduler_init_gpio(scheduler_ctx_t *ctx);
static esp_err_t scheduler_validate_slots(const scheduler_slot_t *slots, size_t slot_count);
static void      scheduler_dispense(scheduler_ctx_t *ctx);

esp_err_t scheduler_start(const scheduler_config_t *config, scheduler_handle_t *out_handle)
{
    esp_err_t        err;
    scheduler_ctx_t *ctx;
    bool             time_valid = false;

    if (!config || !config->rtc_handle || !config->drum_handle || !out_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = scheduler_validate_slots(config->slots, config->slot_count);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_handle = NULL;
    ctx         = calloc(1, sizeof(scheduler_ctx_t));
    if (!ctx)
    {
        return ESP_ERR_NO_MEM;
    }

    ctx->rtc          = config->rtc_handle;
    ctx->drum         = config->drum_handle;
    ctx->int_gpio_num = config->int_gpio_num;
    ctx->slot_count   = config->slot_count;

    ctx->slots        = calloc(config->slot_count, sizeof(scheduler_slot_t));
    ctx->alarm_signal = xSemaphoreCreateBinary();
    ctx->lock         = xSemaphoreCreateMutex();

    if (!ctx->slots || !ctx->alarm_signal || !ctx->lock)
    {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    for (size_t i = 0; i < config->slot_count; i++)
    {
        ctx->slots[i] = config->slots[i];
    }

    if (rv3028_is_time_valid(ctx->rtc, &time_valid) == ESP_OK && !time_valid)
    {
        ESP_LOGW(TAG, "RTC time is not set, the schedule will not line up until it is");
    }

    err = scheduler_arm_next(ctx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to arm the first alarm (%s)", esp_err_to_name(err));
        goto fail;
    }

    err = scheduler_init_gpio(ctx);
    if (err != ESP_OK)
    {
        goto fail;
    }

    if (xTaskCreate(scheduler_task, "scheduler",
                    config->task_stack_size ? config->task_stack_size : SCHEDULER_DEFAULT_STACK_SIZE, ctx,
                    config->task_priority ? (UBaseType_t) config->task_priority : SCHEDULER_DEFAULT_PRIORITY,
                    &ctx->task) != pdPASS)
    {
        gpio_isr_handler_remove(ctx->int_gpio_num);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG, "Started with %u slots, INT on GPIO%d", (unsigned) ctx->slot_count, ctx->int_gpio_num);

    *out_handle = ctx;
    return ESP_OK;

fail:
    if (ctx->lock)
    {
        vSemaphoreDelete(ctx->lock);
    }

    if (ctx->alarm_signal)
    {
        vSemaphoreDelete(ctx->alarm_signal);
    }

    free(ctx->slots);
    free(ctx);
    return err;
}

esp_err_t scheduler_reschedule(scheduler_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return scheduler_arm_next(handle);
}

esp_err_t scheduler_get_next_slot(scheduler_handle_t handle, scheduler_slot_t *out_slot)
{
    if (!handle || !out_slot)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    *out_slot = handle->next_slot;
    xSemaphoreGive(handle->lock);

    return ESP_OK;
}

esp_err_t scheduler_get_slots(scheduler_handle_t handle, const scheduler_slot_t **out_slots, size_t *out_slot_count)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (out_slots)
    {
        *out_slots = handle->slots;
    }

    if (out_slot_count)
    {
        *out_slot_count = handle->slot_count;
    }

    return ESP_OK;
}

static esp_err_t scheduler_validate_slots(const scheduler_slot_t *slots, size_t slot_count)
{
    if (!slots || slot_count == 0 || slot_count > SCHEDULER_MAX_SLOTS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < slot_count; i++)
    {
        if (slots[i].hour > 23 || slots[i].minute > 59)
        {
            return ESP_ERR_INVALID_ARG;
        }

        /* Ascending order is what makes picking the next slot a single pass */
        if (i > 0 && (slots[i].hour * 60 + slots[i].minute) <= (slots[i - 1].hour * 60 + slots[i - 1].minute))
        {
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

static esp_err_t scheduler_init_gpio(scheduler_ctx_t *ctx)
{
    esp_err_t err;

    /*
     * The RV-3028 INT output is open drain, held high by a 4k7 pull-up on the
     * board; the internal pull-up would only fight it, so leave it off and
     * watch for the RTC pulling the line down.
     */
    gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << ctx->int_gpio_num,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };

    err = gpio_config(&int_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure GPIO%d (%s)", ctx->int_gpio_num, esp_err_to_name(err));
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install the GPIO ISR service (%s)", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(ctx->int_gpio_num, scheduler_isr, ctx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to hook the RTC interrupt (%s)", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static void IRAM_ATTR scheduler_isr(void *arg)
{
    scheduler_ctx_t *ctx          = arg;
    BaseType_t       higher_woken = pdFALSE;

    xSemaphoreGiveFromISR(ctx->alarm_signal, &higher_woken);

    if (higher_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

/*
 * Both the interrupt and the periodic recheck end up in the same place: read
 * the alarm flag over I2C and act on it. The flag, not the edge, is what says
 * an alarm actually happened, so a spurious wake-up costs one register read.
 */
static void scheduler_task(void *arg)
{
    scheduler_ctx_t *ctx = arg;

    for (;;)
    {
        xSemaphoreTake(ctx->alarm_signal, pdMS_TO_TICKS(SCHEDULER_RECHECK_MS));

        bool      triggered = false;
        esp_err_t err       = rv3028_get_alarm_flag(ctx->rtc, &triggered);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read the alarm flag (%s)", esp_err_to_name(err));
            continue;
        }

        if (!triggered)
        {
            continue;
        }

        scheduler_dispense(ctx);

        /* Arming the next slot clears the flag, which releases INT for the next edge */
        err = scheduler_arm_next(ctx);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to arm the next alarm (%s)", esp_err_to_name(err));

            /* Clear it anyway, otherwise INT stays low and no further alarm is ever seen */
            rv3028_clear_alarm_flag(ctx->rtc);
        }
    }
}

static void scheduler_dispense(scheduler_ctx_t *ctx)
{
    dispenser_result_t result;

    esp_err_t err = dispenser_advance(ctx->drum, &result);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Scheduled dispense failed (%s)", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Scheduled dispense done in %lu ms", (unsigned long) result.elapsed_ms);
}

/*
 * The RV-3028 alarm matches on hour and minute with the day masked off, so it
 * repeats daily. Three times a day therefore means re-arming it for the next
 * slot every time one fires, which is also what keeps the schedule pinned to
 * the wall clock rather than drifting from an interval.
 */
static esp_err_t scheduler_arm_next(scheduler_ctx_t *ctx)
{
    esp_err_t err;
    struct tm now;
    size_t    index = 0;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);

    err = rv3028_get_time(ctx->rtc, &now);
    if (err == ESP_OK)
    {
        int now_minutes = now.tm_hour * 60 + now.tm_min;

        /* The first slot still ahead today, or wrap round to the first one tomorrow */
        for (size_t i = 0; i < ctx->slot_count; i++)
        {
            if ((ctx->slots[i].hour * 60 + ctx->slots[i].minute) > now_minutes)
            {
                index = i;
                break;
            }
        }

        err = rv3028_set_alarm(ctx->rtc, ctx->slots[index].hour, ctx->slots[index].minute);
    }

    if (err == ESP_OK)
    {
        ctx->next_slot = ctx->slots[index];
    }

    xSemaphoreGive(ctx->lock);

    return err;
}
