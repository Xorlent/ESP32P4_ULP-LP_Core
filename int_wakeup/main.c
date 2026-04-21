#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "../../src/programs/ulp_shared.h"

static volatile ulp_shared_mem_t * const shared =
    (volatile ulp_shared_mem_t *)ULP_SHARED_MEM_ADDR;

static lp_io_num_t s_target_pin;

// PMU_SLP_WAKEUP_CNTL2_REG - Bit 1 enables the PMU software wakeup source
#define PMU_SLP_WAKEUP_CNTL2  (*(volatile uint32_t *)0x50115128u)

void LP_CORE_ISR_ATTR ulp_lp_core_lp_io_intr_handler(void)
{
    PMU_SLP_WAKEUP_CNTL2 |= (1u << 1);

    ulp_lp_core_gpio_wakeup_disable(s_target_pin);
    ulp_lp_core_gpio_clear_intr_status();

    /* Latch a single LP IO wake event before handing control back to HP. */
    shared->lp_counter++;
    shared->data[0] = ulp_lp_core_gpio_get_level(s_target_pin);
    shared->status |= ULP_STATUS_WAKEUP_PENDING;

    ulp_lp_core_wakeup_main_processor();
    ulp_lp_core_halt();
}

int main(void)
{
    PMU_SLP_WAKEUP_CNTL2 |= (1u << 1);

    shared->status |= ULP_STATUS_RUNNING;

    s_target_pin = (lp_io_num_t)(shared->config0 & 0x0F);
    const gpio_int_type_t intr_type = (gpio_int_type_t)(shared->config1 & 0x07);

    /* HP owns the LP IO mux and pull configuration.
     * LP only re-applies input mode before arming the interrupt path. */
    ulp_lp_core_gpio_init(s_target_pin);
    ulp_lp_core_gpio_input_enable(s_target_pin);
    ulp_lp_core_gpio_output_disable(s_target_pin);
    ulp_lp_core_gpio_clear_intr_status();
    ulp_lp_core_gpio_wakeup_enable(s_target_pin, intr_type);
    ulp_lp_core_gpio_intr_enable(s_target_pin, intr_type);
    ulp_lp_core_intr_enable();

    while (true) {
        ulp_lp_core_wait_for_intr();
    }

    return 0;
}
