#include "py/mphal.h"
#include "sdram.h"

void board_low_power(int mode)
{
    switch (mode) {
        case 0:     // Leave stop mode.
            sdram_leave_low_power();
            break;
        case 1:     // Enter stop mode.
            sdram_enter_low_power();
            break;
        case 2:     // Enter standby mode.
            sdram_powerdown();
            break;
    }
}
