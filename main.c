#include <stdio.h>
#include "ztimer.h"

#include "soniclib.h"
#include "ch_extra_display_utils.h"
#include "soniclib_params.h"

#ifdef SHORT_RANGE
#include "ch101_gpr_sr.h"
#define FW_INIT_FUNC ch101_gpr_sr_init
#else
#include "ch101_gpr.h"
#define FW_INIT_FUNC ch101_gpr_init
#endif

#define SENSOR_MAX_RANGE_MM (120)   // maximum range, in mm
#define CH101_MAX_SAMPLES   (225)
#define TRIGGER             GPIO_PIN(PA, 25)

#define MEASURE_PENDING     (1 << 0)
#define DATA_READY_FLAG     (1 << 1)

volatile uint32_t taskflags = 0;

static ch_iq_sample_t iq_data[CH101_MAX_SAMPLES];

static uint32_t active_devices;
static uint32_t data_ready_devices;
static uint8_t  num_connected_sensors = 0;

static void trigger_callback(void *arg)
{
    (void)arg;
    if (taskflags == 0) {
        taskflags |= MEASURE_PENDING;
        ch_group_trigger(&soniclib_group);
    }
}

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type) {
    (void)int_type;
    data_ready_devices |= (1 << dev_num);       // add to data-ready bit mask
    if (data_ready_devices == active_devices) {
        // All active sensors have interrupted after performing a measurement
        data_ready_devices = 0;
        // Set data-ready flag - it will be checked in main() loop
        taskflags |= DATA_READY_FLAG;
        chdrv_int_group_interrupt_disable(grp_ptr);
    }
}

static void handle_data_ready(ch_group_t *grp_ptr) {
    uint8_t num_ports = ch_get_num_ports(grp_ptr);
    printf("[");
    for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        printf("{\"sensor\": %d", dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            uint32_t range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ROUND_TRIP);
            if (range == CH_NO_TARGET) {
                printf(", \"range_mm\": -1, \"amp\": -1");
            } else {
                uint16_t amplitude = ch_get_amplitude(dev_ptr);
                printf(", \"range_mm\": %0.1f, \"amp\": %u", (float) range/32.0f, amplitude);
            }
            uint16_t num_samples = ch_get_num_samples(dev_ptr);
            printf(", \"num_samples\": %d", num_samples);
            uint8_t res = ch_get_iq_data(dev_ptr, iq_data, 0, num_samples, CH_IO_MODE_BLOCK);
            if (res == 0) {
                printf(", \"qi_data\": [");
                ch_iq_sample_t *iq_ptr;
                iq_ptr = iq_data;
                for (int count = 0; count < num_samples; count++) {
                    printf("[%d,%d]", iq_ptr->q, iq_ptr->i);
                    if (count < num_samples-1) { printf(","); }
                    iq_ptr++;
                }
                printf("]");
            }
        }
        printf("}");
        if (dev_num < num_ports-1) { printf(","); }
    }
    printf("]\n");
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
    gpio_init_int(TRIGGER, GPIO_IN, GPIO_FALLING, trigger_callback, NULL);

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

    // Register callback function for measure ready interrupt
    ch_io_int_callback_set(grp_ptr, sensor_int_callback);

    printf ("Configuring sensor(s)...\n");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;            // count one more connected
            active_devices |= (1 << dev_num);   // add to active device bit mask
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

    printf("Starting measures\n\n");
    while (1) {
        if (taskflags == 0) {
            ztimer_sleep(ZTIMER_MSEC, 1);
        }
        if (taskflags & DATA_READY_FLAG) {
            handle_data_ready(grp_ptr);
            taskflags = 0; // now we can start another measure
        }
    }
    return 0;
}
