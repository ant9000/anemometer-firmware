#include <stdio.h>
#include "soniclib.h"
#include "ch_extra_display_utils.h"
#include "soniclib_params.h"
#include "ch101_gpr.h"

#define FW_INIT_FUNC ch101_gpr_init
#define SENSOR_MAX_RANGE_MM       (120)   /* maximum range, in mm */
#define READ_IQ_DATA
#define READ_IQ_BLOCKING

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags
 *   that are set in I/O processing routines.  The flags are checked in the
 *   main() loop and, if set, will cause an appropriate handler function to
 *   be called to process sensor data.
 */
volatile uint32_t taskflags = 0;

/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG     (1 << 0)
#define IQ_READY_FLAG       (1 << 1)

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;
/* Number of connected sensors */
static uint8_t  num_connected_sensors = 0;
/* Number of sensors that use h/w triggering to start measurement */
static uint8_t  num_triggered_devices = 0;

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type) {
    (void)int_type;
    data_ready_devices |= (1 << dev_num);       // add to data-ready bit mask
    if (data_ready_devices == active_devices) {
        /* All active sensors have interrupted after performing a measurement */
        data_ready_devices = 0;
        /* Set data-ready flag - it will be checked in main() loop */
        taskflags |= DATA_READY_FLAG;
        chdrv_int_group_interrupt_disable(grp_ptr);
    }
}

static void io_complete_callback(ch_group_t __attribute__((unused)) *grp_ptr) {
    taskflags |= IQ_READY_FLAG;
}

int main(void)
{
    ch_group_t *grp_ptr = &soniclib_group;
    uint8_t res;
    uint8_t num_ports;
    uint8_t dev_num;

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
    num_ports = ch_get_num_ports(grp_ptr);
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(soniclib_devices[dev_num]);
        res |= ch_init(dev_ptr, grp_ptr, dev_num, FW_INIT_FUNC);
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

    printf("Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\n");

    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            printf("%d\tCH%d\t %u Hz\t%u@%ums\t%s\n", dev_num,
                                    ch_get_part_number(dev_ptr),
                                    (unsigned int) ch_get_frequency(dev_ptr),
                                    ch_get_rtc_cal_result(dev_ptr),
                                    ch_get_rtc_cal_pulselength(dev_ptr),
                                    ch_get_fw_version_string(dev_ptr));
        }
    }
    printf("\n");

    /* Register callback function for measure ready interrupt */
    ch_io_int_callback_set(grp_ptr, sensor_int_callback);

    /* Register callback function for I/Q data ready event */
    ch_io_complete_callback_set(grp_ptr, io_complete_callback);

    printf ("Configuring sensor(s)...\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;            // count one more connected
            active_devices |= (1 << dev_num);   // add to active device bit mask
            num_triggered_devices++;            // will be triggered
            dev_config.mode            = CH_MODE_TRIGGERED_TX_RX;
            dev_config.max_range       = SENSOR_MAX_RANGE_MM;
            dev_config.sample_interval = 0;
            res = ch_set_config(dev_ptr, &dev_config);
            if (!res) {
                ch_extra_display_config_info(dev_ptr);
            } else {
                printf("Device %d: Error during ch_set_config()\n", dev_num);
            }
        }
    }

    return 0;
}
