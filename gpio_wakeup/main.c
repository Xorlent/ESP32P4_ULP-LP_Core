#include <stdint.h>
#include "ulp_lp_core_utils.h"      /* ulp_lp_core_wakeup_main_processor / halt */
#include "ulp_lp_core_gpio.h"       /* ulp_lp_core_gpio_get_level               */
#include "../../src/programs/ulp_shared.h"

static volatile ulp_shared_mem_t * const shared =
    (volatile ulp_shared_mem_t *)ULP_SHARED_MEM_ADDR;

/* Debounce state */
static uint32_t stable_count = 0;

// PMU_SLP_WAKEUP_CNTL2_REG - Bit 1 enables the PMU software wakeup source
#define PMU_SLP_WAKEUP_CNTL2  (*(volatile uint32_t *)0x50115128u)

int main(void)
{
    /* Ensure the HP can be woken by lp_trigger_hp */
    PMU_SLP_WAKEUP_CNTL2 |= (1u << 1);

    /* Signal to HP core that we are alive */
    shared->status |= ULP_STATUS_RUNNING;

    /* Read config written by the HP core */
    const lp_io_num_t target_pin   = (lp_io_num_t)(shared->config0 & 0x0F);
    const uint32_t    target_level = shared->config1 & 0x01;
    const uint32_t    debounce     = shared->config2;

    /* HP owns the LP IO mux and pull configuration.
     * Re-apply input enable from LP side so sampling still works after deep sleep. */
    ulp_lp_core_gpio_init(target_pin);
    ulp_lp_core_gpio_input_enable(target_pin);

    /* Sample the GPIO */
    uint32_t level = ulp_lp_core_gpio_get_level(target_pin);
    shared->data[0] = level;
    shared->lp_counter++;

    if (level == target_level) {
        if (debounce == 0 || ++stable_count >= debounce) {
            /* Latch a single wake request, then halt until the HP side re-arms us. */
            shared->status |= ULP_STATUS_WAKEUP_PENDING;
            ulp_lp_core_wakeup_main_processor();
            ulp_lp_core_halt();
        }
    } else {
        stable_count = 0;
    }

    /* Return to caller; lp_core_startup re-arms the LP timer and sleeps.
     * The LP timer fires and calls main() again for the next poll cycle. */
    return 0;
}
