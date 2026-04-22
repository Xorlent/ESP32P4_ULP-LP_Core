#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_utils.h"
#include "../../src/programs/ulp_shared.h"

static volatile ulp_shared_mem_t * const shared =
    (volatile ulp_shared_mem_t *)ULP_SHARED_MEM_ADDR;

/* These pads seem to remain held across deep sleep unless the LP side
 * explicitly releases them before driving the software-I2C bus. */
extern void pmu_enable_unhold_pads(void);

// PMU_SLP_WAKEUP_CNTL2_REG - Bit 1 enables the PMU software wakeup source
#define PMU_SLP_WAKEUP_CNTL2  (*(volatile uint32_t *)0x50115128u)
#define SOFT_I2C_MAX_LP_IO_NUM            15u

//Timings
#define SOFT_I2C_HALF_PERIOD_US           5u
#define SOFT_I2C_SCL_WAIT_RETRIES         64u
#define SOFT_I2C_POWER_SETTLE_DELAY_US    2000u
// SHT4X sensor constants
#define SHT4X_I2C_ADDR_7BIT               0x44u
#define SHT4X_I2C_WRITE_ADDR              ((SHT4X_I2C_ADDR_7BIT << 1) | 0u)
#define SHT4X_I2C_READ_ADDR               ((SHT4X_I2C_ADDR_7BIT << 1) | 1u)
#define SHT4X_CMD_MEASURE_HIGH_REPEAT     0xFDu
#define SHT4X_MEASURE_DELAY_US            9000u

enum {
    SOFT_I2C_TEMP_STATUS_OK = 0,
    SOFT_I2C_TEMP_STATUS_BUS_NOT_IDLE = 1,
    SOFT_I2C_TEMP_STATUS_CMD_ADDR_NACK = 2,
    SOFT_I2C_TEMP_STATUS_CMD_NACK = 3,
    SOFT_I2C_TEMP_STATUS_READ_ADDR_NACK = 4,
    SOFT_I2C_TEMP_STATUS_TEMP_CRC_FAIL = 5,
    SOFT_I2C_TEMP_STATUS_HUM_CRC_FAIL = 6,
    SOFT_I2C_TEMP_STATUS_STOP_FAILED = 7,
};

static lp_io_num_t s_sda_pin;
static lp_io_num_t s_scl_pin;
static uint16_t s_power_pin;

static bool s_power_gpio_enabled(void)
{
    return s_power_pin <= SOFT_I2C_MAX_LP_IO_NUM;
}

static void s_power_gpio_init(void)
{
    if (!s_power_gpio_enabled()) {
        return;
    }

    const lp_io_num_t power_pin = (lp_io_num_t)s_power_pin;
    ulp_lp_core_gpio_init(power_pin);
    ulp_lp_core_gpio_output_enable(power_pin);
    ulp_lp_core_gpio_set_output_mode(power_pin, RTCIO_LL_OUTPUT_NORMAL);
    ulp_lp_core_gpio_pullup_disable(power_pin);
    ulp_lp_core_gpio_pulldown_disable(power_pin);
    ulp_lp_core_gpio_set_level(power_pin, 0);
}

static void s_power_gpio_set(bool enabled)
{
    if (!s_power_gpio_enabled()) {
        return;
    }

    ulp_lp_core_gpio_set_level((lp_io_num_t)s_power_pin, enabled ? 1 : 0);
}

static void s_i2c_delay(void)
{
    ulp_lp_core_delay_us(SOFT_I2C_HALF_PERIOD_US);
}

static void s_sda_release(void)
{
    ulp_lp_core_gpio_set_level(s_sda_pin, 1);
}

static void s_sda_drive_low(void)
{
    ulp_lp_core_gpio_set_level(s_sda_pin, 0);
}

static void s_scl_release(void)
{
    ulp_lp_core_gpio_set_level(s_scl_pin, 1);
}

static void s_scl_drive_low(void)
{
    ulp_lp_core_gpio_set_level(s_scl_pin, 0);
}

static bool s_scl_wait_high(void)
{
    s_scl_release();
    for (uint32_t retry = 0; retry < SOFT_I2C_SCL_WAIT_RETRIES; ++retry) {
        if (ulp_lp_core_gpio_get_level(s_scl_pin) != 0) {
            return true;
        }
        s_i2c_delay();
    }
    return false;
}

static bool s_i2c_stop(void)
{
    s_sda_drive_low();
    s_i2c_delay();
    if (!s_scl_wait_high()) {
        return false;
    }
    s_i2c_delay();
    s_sda_release();
    s_i2c_delay();
    return (ulp_lp_core_gpio_get_level(s_sda_pin) != 0) &&
           (ulp_lp_core_gpio_get_level(s_scl_pin) != 0);
}

static bool s_i2c_start(void)
{
    s_sda_release();
    if (!s_scl_wait_high()) {
        return false;
    }
    s_i2c_delay();
    if ((ulp_lp_core_gpio_get_level(s_sda_pin) == 0) ||
        (ulp_lp_core_gpio_get_level(s_scl_pin) == 0)) {
        return false;
    }
    s_sda_drive_low();
    s_i2c_delay();
    s_scl_drive_low();
    s_i2c_delay();
    return true;
}

static bool s_i2c_write_bit(uint8_t bit)
{
    if (bit != 0) {
        s_sda_release();
    } else {
        s_sda_drive_low();
    }

    s_i2c_delay();
    if (!s_scl_wait_high()) {
        return false;
    }
    s_i2c_delay();
    s_scl_drive_low();
    s_i2c_delay();
    return true;
}

static bool s_i2c_read_bit(uint8_t *bit)
{
    s_sda_release();
    s_i2c_delay();
    if (!s_scl_wait_high()) {
        return false;
    }
    s_i2c_delay();
    *bit = (uint8_t)(ulp_lp_core_gpio_get_level(s_sda_pin) != 0);
    s_scl_drive_low();
    s_i2c_delay();
    return true;
}

static bool s_i2c_write_byte(uint8_t value, bool *acked)
{
    for (int bit = 7; bit >= 0; --bit) {
        if (!s_i2c_write_bit((uint8_t)((value >> bit) & 0x01u))) {
            return false;
        }
    }

    uint8_t nack = 1;
    if (!s_i2c_read_bit(&nack)) {
        return false;
    }

    *acked = (nack == 0);
    return true;
}

static bool s_i2c_read_byte(uint8_t *value, bool ack_after)
{
    uint8_t result = 0;
    for (int bit = 7; bit >= 0; --bit) {
        uint8_t sampled = 0;
        if (!s_i2c_read_bit(&sampled)) {
            return false;
        }
        result |= (uint8_t)(sampled << bit);
    }

    if (!s_i2c_write_bit(ack_after ? 0u : 1u)) {
        return false;
    }

    *value = result;
    return true;
}

static uint8_t s_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFFu;
    for (uint32_t index = 0; index < len; ++index) {
        crc ^= data[index];
        for (uint32_t bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80u) != 0) {
                crc = (uint8_t)((crc << 1) ^ 0x31u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void s_i2c_bus_init(lp_io_num_t sda_pin, lp_io_num_t scl_pin)
{
    s_sda_pin = sda_pin;
    s_scl_pin = scl_pin;

    ulp_lp_core_gpio_init(s_sda_pin);
    ulp_lp_core_gpio_input_enable(s_sda_pin);
    ulp_lp_core_gpio_output_enable(s_sda_pin);
    ulp_lp_core_gpio_set_output_mode(s_sda_pin, RTCIO_LL_OUTPUT_OD);
    ulp_lp_core_gpio_pullup_disable(s_sda_pin);
    ulp_lp_core_gpio_pulldown_disable(s_sda_pin);

    ulp_lp_core_gpio_init(s_scl_pin);
    ulp_lp_core_gpio_input_enable(s_scl_pin);
    ulp_lp_core_gpio_output_enable(s_scl_pin);
    ulp_lp_core_gpio_set_output_mode(s_scl_pin, RTCIO_LL_OUTPUT_OD);
    ulp_lp_core_gpio_pullup_disable(s_scl_pin);
    ulp_lp_core_gpio_pulldown_disable(s_scl_pin);

    s_sda_release();
    s_scl_release();
    s_i2c_delay();
}

static uint32_t s_sht4x_measure(uint16_t *raw_temp,
                                uint16_t *raw_humidity)
{
    uint8_t buffer[6] = {0};
    bool acked = false;

    if (!s_i2c_start()) {
        return SOFT_I2C_TEMP_STATUS_BUS_NOT_IDLE;
    }

    if (!s_i2c_write_byte(SHT4X_I2C_WRITE_ADDR, &acked) || !acked) {
        s_i2c_stop();
        return SOFT_I2C_TEMP_STATUS_CMD_ADDR_NACK;
    }

    if (!s_i2c_write_byte(SHT4X_CMD_MEASURE_HIGH_REPEAT, &acked) || !acked) {
        s_i2c_stop();
        return SOFT_I2C_TEMP_STATUS_CMD_NACK;
    }

    if (!s_i2c_stop()) {
        return SOFT_I2C_TEMP_STATUS_STOP_FAILED;
    }

    ulp_lp_core_delay_us(SHT4X_MEASURE_DELAY_US);

    if (!s_i2c_start()) {
        return SOFT_I2C_TEMP_STATUS_BUS_NOT_IDLE;
    }

    if (!s_i2c_write_byte(SHT4X_I2C_READ_ADDR, &acked) || !acked) {
        s_i2c_stop();
        return SOFT_I2C_TEMP_STATUS_READ_ADDR_NACK;
    }

    for (uint32_t index = 0; index < 6; ++index) {
        if (!s_i2c_read_byte(&buffer[index], index < 5)) {
            s_i2c_stop();
            return SOFT_I2C_TEMP_STATUS_STOP_FAILED;
        }
    }

    if (!s_i2c_stop()) {
        return SOFT_I2C_TEMP_STATUS_STOP_FAILED;
    }

    if (s_crc8(buffer, 2) != buffer[2]) {
        return SOFT_I2C_TEMP_STATUS_TEMP_CRC_FAIL;
    }

    if (s_crc8(&buffer[3], 2) != buffer[5]) {
        return SOFT_I2C_TEMP_STATUS_HUM_CRC_FAIL;
    }

    *raw_temp = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    *raw_humidity = (uint16_t)(((uint16_t)buffer[3] << 8) | buffer[4]);
    return SOFT_I2C_TEMP_STATUS_OK;
}

int main(void)
{
    // Unhold is required to get software I2C to work on these pins, requires external pull-ups (10kOhm used in testing)
    pmu_enable_unhold_pads();
    PMU_SLP_WAKEUP_CNTL2 |= (1u << 1);
    shared->status |= ULP_STATUS_RUNNING;
    shared->lp_counter++;

    const lp_io_num_t sda_pin = (lp_io_num_t)ULP_UNPACK_U16_PAIR_LOW(shared->config0);
    const lp_io_num_t scl_pin = (lp_io_num_t)ULP_UNPACK_U16_PAIR_HIGH(shared->config0);
    s_power_pin = ULP_UNPACK_U16_PAIR_LOW(shared->config1);
    const uint16_t raw_temp_low_limit = ULP_UNPACK_U16_PAIR_LOW(shared->config2);
    const uint16_t raw_temp_high_limit = ULP_UNPACK_U16_PAIR_HIGH(shared->config2);
    const uint16_t raw_humidity_low_limit = ULP_UNPACK_U16_PAIR_LOW(shared->config3);
    const uint16_t raw_humidity_high_limit = ULP_UNPACK_U16_PAIR_HIGH(shared->config3);

    s_power_gpio_init();
    s_power_gpio_set(true);
    ulp_lp_core_delay_us(SOFT_I2C_POWER_SETTLE_DELAY_US);

    s_i2c_bus_init(sda_pin, scl_pin);

    uint16_t raw_temp = 0;
    uint16_t raw_humidity = 0;
    const uint32_t status = s_sht4x_measure(&raw_temp, &raw_humidity);

    s_power_gpio_set(false);

    shared->data[1] = status;

    if (status != SOFT_I2C_TEMP_STATUS_OK) {
        return 0;
    }

    shared->data[0] = raw_temp;
    shared->data[2] = raw_humidity;

    const bool temp_enabled = raw_temp_low_limit <= raw_temp_high_limit;
    const bool humidity_enabled = raw_humidity_low_limit <= raw_humidity_high_limit;

    const bool temp_triggered = temp_enabled &&
        ((raw_temp < raw_temp_low_limit) || (raw_temp > raw_temp_high_limit));
    const bool humidity_triggered = humidity_enabled &&
        ((raw_humidity < raw_humidity_low_limit) ||
         (raw_humidity > raw_humidity_high_limit));

    if (temp_triggered || humidity_triggered) {
        shared->status |= ULP_STATUS_WAKEUP_PENDING;
        ulp_lp_core_wakeup_main_processor();
        ulp_lp_core_halt();
    }

    return 0;
}
