#include <stdio.h>
#include "soniclib.h"
#include "soniclib_params.h"
#include "ch101/ch101_gpr.h"

#define FW_INIT_FUNC ch101_gpr_init

ch_dev_t chirp_devices[SONICLIB_NUMOF];
ch_group_t chirp_group;

int main(void)
{
    ch_group_t *grp_ptr = &chirp_group;
    uint8_t res;
    uint8_t num_devices;

	printf("TDK InvenSense\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);

    res = ch_group_init(grp_ptr, SONICLIB_NUMOF, SONICLIB_NUMOF, SONICLIB_RTC_CAL_PULSE_MS);
    printf("ch_group_init: %d\n", res);

    for (uint8_t i = 0; i < SONICLIB_NUMOF; i++) {
        gpio_init(soniclib_params[i].prog_pin, GPIO_OUT);
    }

    printf("Initializing sensor(s)...\n");
    res = 0;
    num_devices = ch_get_num_ports(grp_ptr);
    for (int i = 0; i < num_devices; i++) {
        ch_dev_t *dev_ptr = &(chirp_devices[i]);
        res |= ch_init(dev_ptr, grp_ptr, i, FW_INIT_FUNC);
    }
    if (res == 0) {
        printf("starting group...\n");
        res = ch_group_start(grp_ptr);
    }

    if (res == 0) {
        printf("OK\n");
    } else {
        printf("FAILED: %d\n", res);
    }
    printf("\n");

    return 0;
}
