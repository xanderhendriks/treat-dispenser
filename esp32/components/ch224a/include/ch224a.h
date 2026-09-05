#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ch224a_t *ch224a_handle_t;

/* 7-bit addresses the chip can answer on (datasheet section 5.2.3) */
#define CH224A_I2C_ADDRESS_PRIMARY   0x22
#define CH224A_I2C_ADDRESS_SECONDARY 0x23

/* Register addresses (datasheet table 5-3) */
#define CH224A_REG_STATUS        0x09 /* read-only  */
#define CH224A_REG_VOLTAGE       0x0A /* write-only */
#define CH224A_REG_CURRENT       0x50 /* read-only  */
#define CH224A_REG_AVS_HIGH      0x51 /* write-only, CH224Q only */
#define CH224A_REG_AVS_LOW       0x52 /* write-only, CH224Q only */
#define CH224A_REG_PPS           0x53 /* write-only, CH224Q only */
#define CH224A_REG_PD_DATA_FIRST 0x60 /* read-only, CH224Q only  */
#define CH224A_REG_PD_DATA_LAST  0x8F

/* Status register (0x09) bit masks */
#define CH224A_STATUS_BC         0x01 /* BC 1.2 handshake succeeded */
#define CH224A_STATUS_QC2        0x02 /* QC2.0 handshake succeeded  */
#define CH224A_STATUS_QC3        0x04 /* QC3.0 handshake succeeded  */
#define CH224A_STATUS_PD         0x08 /* USB PD handshake succeeded */
#define CH224A_STATUS_EPR_ACTIVE 0x10 /* EPR mode active            */
#define CH224A_STATUS_EPR_EXISTS 0x20 /* Supply offers EPR (>100 W) */
#define CH224A_STATUS_AVS_EXISTS 0x40 /* Supply offers AVS          */

/* The current register counts in 50 mA steps */
#define CH224A_CURRENT_STEP_MA 50

    /* Requested voltage gear written to CH224A_REG_VOLTAGE. The PPS (6) and
     * AVS (7) gears are CH224Q only and are deliberately not listed here. */
    typedef enum
    {
        CH224A_VOLTAGE_5V  = 0,
        CH224A_VOLTAGE_9V  = 1,
        CH224A_VOLTAGE_12V = 2,
        CH224A_VOLTAGE_15V = 3,
        CH224A_VOLTAGE_20V = 4,
        CH224A_VOLTAGE_28V = 5,
    } ch224a_voltage_t;

    /* One entry of a full register dump. Every register is read on its own, so
     * a register that does not answer only fails its own entry. */
    typedef struct
    {
        uint8_t   address;
        uint8_t   value;
        esp_err_t err; /* ESP_OK when value holds a byte read from the chip */
    } ch224a_register_t;

/* Number of entries ch224a_read_all_registers() fills in: the six named
 * registers plus the 0x60-0x8F PD power data block. */
#define CH224A_REGISTER_COUNT (6 + (CH224A_REG_PD_DATA_LAST - CH224A_REG_PD_DATA_FIRST + 1))

    typedef struct
    {
        i2c_master_bus_handle_t bus;

        uint8_t  i2c_address;        /* 0 probes 0x22 and then 0x23 */
        uint32_t i2c_clock_speed_hz; /* 0 selects the 400 kHz default */

        /* Highest gear ch224a_set_voltage() is allowed to request. This is a
         * board property rather than a chip property: where the CH224A output
         * feeds the whole supply rail, anything above what the input TVS and
         * the buck converter tolerate must never be requested. Note that a
         * zero-initialized config therefore pins the chip to 5V. */
        ch224a_voltage_t max_voltage;
    } ch224a_config_t;

    /**
     * Initialize the CH224A USB-PD sink controller driver.
     *
     * Adds the chip to an existing I2C master bus and verifies communication by
     * reading the status register. When config->i2c_address is 0 both documented
     * addresses are tried in turn.
     *
     * The I2C interface is only enabled when the chip is strapped for single
     * resistor configuration (a resistor from CFG1 to GND); with I/O level
     * configuration the SCL and SDA pins act as CFG2 and CFG3 instead and the
     * chip will not answer.
     *
     * @param config Driver configuration.
     * @param out_handle Receives the driver handle on success.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND when the chip does not
     *         answer, or an error from the I2C driver.
     */
    esp_err_t ch224a_init(const ch224a_config_t *config, ch224a_handle_t *out_handle);

    /**
     * Get the 7-bit I2C address the chip answered on.
     *
     * @param handle Driver handle.
     * @param out_address Receives the address.
     * @return ESP_OK on success.
     */
    esp_err_t ch224a_get_address(ch224a_handle_t handle, uint8_t *out_address);

    /**
     * Read a single register.
     *
     * The CH224A only supports single byte transfers, so this is the only read
     * primitive; the range and dump helpers below repeat it per register.
     *
     * @param handle Driver handle.
     * @param reg Register address.
     * @param out_value Receives the register value.
     * @return ESP_OK on success or an error from the I2C driver.
     */
    esp_err_t ch224a_read_register(ch224a_handle_t handle, uint8_t reg, uint8_t *out_value);

    /**
     * Write a single register.
     *
     * @param handle Driver handle.
     * @param reg Register address.
     * @param value Value to write.
     * @return ESP_OK on success or an error from the I2C driver.
     */
    esp_err_t ch224a_write_register(ch224a_handle_t handle, uint8_t reg, uint8_t value);

    /**
     * Read a range of consecutive registers, one byte at a time.
     *
     * Stops at and returns the first error encountered.
     *
     * @param handle Driver handle.
     * @param first_reg Address of the first register to read.
     * @param out_values Receives count register values.
     * @param count Number of registers to read.
     * @return ESP_OK on success or an error from the I2C driver.
     */
    esp_err_t ch224a_read_registers(ch224a_handle_t handle, uint8_t first_reg, uint8_t *out_values, size_t count);

    /**
     * Read every documented register into a dump array.
     *
     * Covers the status, voltage, current, AVS and PPS registers plus the
     * 0x60-0x8F PD power data block. The write-only and CH224Q-only registers
     * are read as well: what a CH224A returns for them is not documented, so
     * reading them is only useful during bring-up. Each entry carries its own
     * error code and a failing register does not abort the dump.
     *
     * @param handle Driver handle.
     * @param out_registers Receives up to max_registers entries.
     * @param max_registers Capacity of out_registers, see CH224A_REGISTER_COUNT.
     * @param out_count Receives the number of entries written.
     * @return ESP_OK on success, ESP_ERR_INVALID_SIZE when max_registers is 0.
     */
    esp_err_t ch224a_read_all_registers(ch224a_handle_t handle, ch224a_register_t *out_registers, size_t max_registers,
                                        size_t *out_count);

    /**
     * Get the name of a register for logging.
     *
     * @param reg Register address.
     * @return A static string, "reserved" for undocumented addresses.
     */
    const char *ch224a_register_name(uint8_t reg);

    /**
     * Read the protocol status register (0x09).
     *
     * @param handle Driver handle.
     * @param out_status Receives the status bits, see CH224A_STATUS_*.
     * @return ESP_OK on success.
     */
    esp_err_t ch224a_get_status(ch224a_handle_t handle, uint8_t *out_status);

    /**
     * Read the maximum current the supply offers in the current gear (0x50).
     *
     * Only meaningful once the PD protocol has been negotiated, which
     * ch224a_get_status() reports through CH224A_STATUS_PD.
     *
     * @param handle Driver handle.
     * @param out_current_ma Receives the current limit in milliamps.
     * @return ESP_OK on success.
     */
    esp_err_t ch224a_get_max_current_ma(ch224a_handle_t handle, uint16_t *out_current_ma);

    /**
     * Request an output voltage gear (0x0A).
     *
     * Requests above the configured max_voltage are rejected without touching
     * the chip. The supply only switches when it offers the requested gear.
     *
     * @param handle Driver handle.
     * @param voltage Voltage gear to request.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG when the gear is above
     *         the configured maximum.
     */
    esp_err_t ch224a_set_voltage(ch224a_handle_t handle, ch224a_voltage_t voltage);

    /**
     * Get the voltage gear last requested through ch224a_set_voltage().
     *
     * The voltage register is write-only, so this returns the driver's own copy
     * rather than the chip state. Until a gear has been requested the chip runs
     * on the gear selected by the CFG1 strapping resistor, which the driver
     * cannot read back.
     *
     * @param handle Driver handle.
     * @param out_voltage Receives the voltage gear.
     * @return ESP_OK on success, ESP_ERR_INVALID_STATE when no gear has been
     *         requested since initialization.
     */
    esp_err_t ch224a_get_requested_voltage(ch224a_handle_t handle, ch224a_voltage_t *out_voltage);

    /**
     * Get the nominal millivolts of a voltage gear, for logging.
     *
     * @param voltage Voltage gear.
     * @return Millivolts, 0 for an unknown gear.
     */
    uint32_t ch224a_voltage_to_mv(ch224a_voltage_t voltage);

#ifdef __cplusplus
}
#endif
