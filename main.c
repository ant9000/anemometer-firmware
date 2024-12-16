#include <stdio.h>
#include <string.h>
#include "ztimer.h"
#include "shell.h"

#include "soniclib.h"
#include "ch_extra_display_utils.h"
#include "soniclib_params.h"
#include "persistence.h"

#include "hdc3020.h"
#include "hdc3020_params.h"

extern void auto_init_hdc3020(void);
static hdc3020_t hdc3020_devs[HDC3020_NUMOF];
typedef struct {
    uint8_t connected;
    double temperature;
    double humidity;
} hdc3020_data_t;
static hdc3020_data_t hdc3020_data[HDC3020_NUMOF];

#include "ch101_gpr.h"
#include "ch101_gpr_sr.h"
#include "ch101_gpr_sr_open.h"
#include "ch101_gpr_sr_narrow.h"

#ifndef SENSOR_MAX_RANGE_MM
#define SENSOR_MAX_RANGE_MM (120)   // maximum range, in mm
#endif
#define CH101_MAX_SAMPLES   (225)
#define TRIGGER             GPIO_PIN(PA, 6)

#ifndef DEFAULT_SONICLIB_CFG
#define DEFAULT_SONICLIB_CFG  {.mode=CH_MODE_TRIGGERED_TX_RX, .max_range=SENSOR_MAX_RANGE_MM, .sample_interval=0}
#endif
#ifndef DEFAULT_ROUND_ROBIN
#define DEFAULT_ROUND_ROBIN 0
#endif
#ifndef DEFAULT_FIRMWARE
#define DEFAULT_FIRMWARE "sr"
#endif
#ifndef DEFAULT_RX_PRETRIGGER
#define DEFAULT_RX_PRETRIGGER 0
#endif

#define MEASURE_PENDING     (1 << 0)
#define DATA_READY_FLAG     (1 << 1)

volatile uint32_t taskflags = 0;


static uint32_t active_devices;
static uint32_t data_ready_devices = 0;
static uint8_t  num_connected_sensors = 0;

typedef struct {
    ch_mode_t      mode;
    uint32_t       range;
    uint16_t       amplitude;
    uint16_t       num_samples;
    ch_iq_sample_t iq_data[CH101_MAX_SAMPLES];
} soniclib_data_t;

static soniclib_data_t soniclib_data[SONICLIB_NUMOF];

typedef struct {
    ch_config_t soniclib[SONICLIB_NUMOF];
    bool round_robin;
    char firmware[10];
    bool rx_pretrigger;
} config_t;
static config_t configuration;

static uint8_t counter = 0;
static void apply_configuration(void)
{
    ch_group_t *grp_ptr = &soniclib_group;
    uint8_t num_ports = ch_get_num_ports(grp_ptr);

    num_connected_sensors = 0;
    active_devices = 0;
    for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;            // count one more connected
            active_devices |= (1 << dev_num);   // add to active device bit mask
            if (configuration.round_robin) {
                configuration.soniclib[dev_num].mode = (counter % SONICLIB_NUMOF == dev_num ? CH_MODE_TRIGGERED_TX_RX : CH_MODE_TRIGGERED_RX_ONLY);
            }
            uint8_t res = ch_set_config(dev_ptr, &(configuration.soniclib[dev_num]));
            if (!res) {
                ch_extra_display_config_info(dev_ptr);
            } else {
                printf("Device %d: Error during ch_set_config()\n", dev_num);
            }
        }
    }
    ch_set_rx_pretrigger(grp_ptr, configuration.rx_pretrigger);
    counter++;
}

static void trigger_callback(void *arg) {
    (void)arg;
    if (taskflags == 0) {
        taskflags |= MEASURE_PENDING;
        for (uint8_t i = 0; i < HDC3020_NUMOF; i++) {
            if (hdc3020_data[i].connected) {
                hdc3020_trigger_on_demand_measurement(&hdc3020_devs[i], 0);
            }
        }
        ch_group_trigger(&soniclib_group);
    }
}

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type) {
    (void)int_type;
    (void)grp_ptr;
    data_ready_devices |= (1 << dev_num);       // add to data-ready bit mask
    if (data_ready_devices == active_devices) {
        // All active sensors have interrupted after performing a measurement
        data_ready_devices = 0;
        // Set data-ready flag - it will be checked in main() loop
        taskflags = DATA_READY_FLAG;
        chdrv_int_group_interrupt_disable(grp_ptr);
    }
}

static void handle_data_ready(ch_group_t *grp_ptr) {
    uint8_t num_ports = ch_get_num_ports(grp_ptr);
    memset(soniclib_data, 0, sizeof(soniclib_data));
    for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            soniclib_data[dev_num].mode = ch_get_mode(dev_ptr);
            if ((configuration.round_robin == 0) || (soniclib_data[dev_num].mode == CH_MODE_TRIGGERED_RX_ONLY)) {
                soniclib_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ROUND_TRIP);
                soniclib_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
                soniclib_data[dev_num].num_samples = ch_get_num_samples(dev_ptr);
                int8_t res = ch_get_iq_data(dev_ptr, soniclib_data[dev_num].iq_data, 0, soniclib_data[dev_num].num_samples, CH_IO_MODE_BLOCK);
                if (res != 0) { soniclib_data[dev_num].num_samples = 0; }
            }
        }
    }
    for (uint8_t i = 0; i < HDC3020_NUMOF; i++) {
        if (hdc3020_data[i].connected) {
            if (hdc3020_fetch_on_demand_measurement(&hdc3020_devs[i], &hdc3020_data[i].temperature, &hdc3020_data[i].humidity) != HDC3020_OK) {
                hdc3020_data[i].temperature = -999;
                hdc3020_data[i].humidity = -999;
            }
        }
    }
}

static void print_data(ch_group_t *grp_ptr) {
    uint8_t num_ports = ch_get_num_ports(grp_ptr);
    uint8_t _printed = 0;
    printf("[");
    for (uint8_t i = 0; i < HDC3020_NUMOF; i++) {
        if (hdc3020_data[i].connected) {
            printf("%s{\"hdc3020\":%d,\"temp\":%.1f,\"rh\":%.1f}", (_printed? ",": ""), i, hdc3020_data[i].temperature, hdc3020_data[i].humidity);
            _printed++;
        }
    }
    for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr) && ((configuration.round_robin == 0) || (soniclib_data[dev_num].mode == CH_MODE_TRIGGERED_RX_ONLY))) {
            printf("%s{\"ch101\":%d,\"mode\":\"%s\"", (_printed? ",": ""), dev_num, (soniclib_data[dev_num].mode == CH_MODE_TRIGGERED_TX_RX ? "TXRX" : "RX"));
            _printed++;
            if (soniclib_data[dev_num].range == CH_NO_TARGET) {
                printf(",\"range_mm\":-1,\"amp\":-1");
            } else {
                printf(",\"range_mm\":%0.1f,\"amp\":%u", (float) soniclib_data[dev_num].range/32.0f, soniclib_data[dev_num].amplitude);
            }
            printf(",\"num_samples\":%d", soniclib_data[dev_num].num_samples);
            printf(",\"i\":[");
            ch_iq_sample_t *iq_ptr = soniclib_data[dev_num].iq_data;
            for (int count = 0; count < soniclib_data[dev_num].num_samples; count++) {
                printf("%d", iq_ptr->i);
                if (count < soniclib_data[dev_num].num_samples-1) { printf(","); }
                iq_ptr++;
            }
            printf("],\"q\":[");
            iq_ptr = soniclib_data[dev_num].iq_data;
            for (int count = 0; count < soniclib_data[dev_num].num_samples; count++) {
                printf("%d", iq_ptr->q);
                if (count < soniclib_data[dev_num].num_samples-1) { printf(","); }
                iq_ptr++;
            }
            printf("]}");
        }
    }
    printf("]\n");
}

static int measure_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    ch_group_t *grp_ptr = &soniclib_group;
    for (uint8_t i = 0; i < HDC3020_NUMOF; i++) {
        if (hdc3020_data[i].connected) {
            hdc3020_trigger_on_demand_measurement(&hdc3020_devs[i], 0);
        }
    }
    ch_group_trigger(&soniclib_group);
    uint32_t counter = 0;
    while (counter < 1000) {
        if (taskflags & DATA_READY_FLAG) {
            handle_data_ready(grp_ptr); // fetch available data
            if (configuration.round_robin)
                apply_configuration();
            taskflags = 0; // now we can start another measure
            print_data(grp_ptr); // print data on console
            break;
        } else {
            ztimer_sleep(ZTIMER_MSEC, 1);
            counter++;
        }
    }
    if (counter == 1000) {
        printf("Timed out!\n");
        return 1;
    }
    return 0;
}

static int config_help (void) {
    printf("config show                        -- show current sensors config\n");
    printf("config set <SENSOR> <MODE> <RANGE> -- set mode and range for sensor; SENSOR in [0,%d]; MODE in [txrx, rx]; RANGE in (0,500)\n", SONICLIB_NUMOF-1);
    printf("config rr <ON/OFF>                 -- alternate TXRX and RX between sensors; ON/OFF in [0,1]\n");
    printf("config fw <FW>                     -- set firmware for sensors; FW in [gpr, sr, open, narrow]\n");
    printf("config pt <ON/OFF>                 -- enable RX pretrigger for receiving sensors; ON/OFF in [0,1]\n");
    printf("config default                     -- reset config to default values\n");
    printf("config load                        -- load config from NVM\n");
    printf("config apply                       -- apply config to sensors\n");
    printf("config save                        -- save config to NVM\n");
    printf("config erase                       -- erase NVM\n");
    return 1;
}

static ch_fw_init_func_t fw_init_func(char *firmware) {
    ch_fw_init_func_t func = NULL;
    if (strcmp(firmware, "gpr") == 0) {
        func = ch101_gpr_init;
    } else if (strcmp(firmware, "sr") == 0) {
        func = ch101_gpr_sr_init;
    } else if (strcmp(firmware, "open") == 0) {
        func = ch101_gpr_sr_open_init;
    } else if (strcmp(firmware, "narrow") == 0) {
        func = ch101_gpr_sr_narrow_init;
    }
    return func;
}

static void config_show (void) {
    for (size_t i=0; i<SONICLIB_NUMOF; i++) {
        ch_config_t *cfg = &configuration.soniclib[i];
        printf(
            "CH101[%d]: mode=%s, range=%d mm\n",
            i,
            (cfg->mode == CH_MODE_TRIGGERED_TX_RX ? "TXRX" : "RX"),
            cfg->max_range
        );
    }
    printf("Round-robin: %d\n", configuration.round_robin);
    printf("Firmware: %s\n", configuration.firmware);
    printf("RX pretrigger: %d\n", configuration.rx_pretrigger);
}

static void config_default(void) {
    ch_config_t cfg = DEFAULT_SONICLIB_CFG;
    for (size_t i=0; i<SONICLIB_NUMOF; i++)
        memcpy(&configuration.soniclib[i], &cfg, sizeof(cfg));
    configuration.round_robin = DEFAULT_ROUND_ROBIN;
    strncpy(configuration.firmware, DEFAULT_FIRMWARE, sizeof(configuration.firmware));
    configuration.rx_pretrigger = DEFAULT_RX_PRETRIGGER;
}

static int config_cmd(int argc, char **argv) {
    if (argc == 1) {
        return config_help();
    } else if (strcmp(argv[1], "show") == 0) {
        // config show
        config_show();
    } else if (strcmp(argv[1], "set") == 0) {
        // config set <sensor> <mode> <range>
        if (argc != 5)
            return config_help();
        int dev_num = atoi(argv[2]);
        if ((dev_num < 0) || (dev_num >= (int)SONICLIB_NUMOF))
            return config_help();
        ch_mode_t mode;
        if ((strcmp(argv[3], "txrx") == 0) || (strcmp(argv[3], "TXRX") == 0)) {
            mode = CH_MODE_TRIGGERED_TX_RX;
        } else if ((strcmp(argv[3], "rx") == 0) || (strcmp(argv[3], "RX") == 0)) {
            mode = CH_MODE_TRIGGERED_RX_ONLY;
        } else
            return config_help();
        int max_range = atoi(argv[4]);
        if ((max_range <= 0) || (max_range >= 500))
            return config_help();
        configuration.soniclib[dev_num].mode = mode;
        configuration.soniclib[dev_num].max_range = max_range;
        printf("Config set.\n");
    } else if (strcmp(argv[1], "rr") == 0) {
        // config rr <0|1>
        if (argc != 3)
            return config_help();
        configuration.round_robin = strcmp(argv[2], "1") == 0;
        printf("Config round-robin set.\n");
    } else if (strcmp(argv[1], "fw") == 0) {
        // config fw <gpr|sr|open|narrow>
        if ((argc != 3) || (fw_init_func(argv[2]) == NULL))
            return config_help();
        strncpy(configuration.firmware, argv[2], sizeof(configuration.firmware));
        printf("Config firmware set. Save config and reboot to use it.\n");
    } else if (strcmp(argv[1], "pt") == 0) {
        // config pt <0|1>
        if (argc != 3)
            return config_help();
        configuration.rx_pretrigger = strcmp(argv[2], "1") == 0;
        printf("Config pretrigger set.\n");
    } else if (strcmp(argv[1], "default") == 0) {
        // config default
        config_default();
        printf("Config reset to default.\n");
    } else if (strcmp(argv[1], "load") == 0) {
        // config load
        int res = load_from_nvm(&configuration, sizeof(configuration));
        if (res == 0) {
            printf("Config loaded from NVM.\n");
        } else {
            printf("[ERROR] Config loading failed: %d.\n", res);
        }
    } else if (strcmp(argv[1], "apply") == 0) {
        // config apply
        counter = 0;
        apply_configuration();
        printf("Config applied.\n");
    } else if (strcmp(argv[1], "save") == 0) {
        // config save
        int res = save_to_nvm(&configuration, sizeof(configuration));
        if (res >= 0) {
            printf("Config saved to NVM.\n");
        } else {
            printf("[ERROR] Config saving failed: %d.\n", res);
        }
    } else if (strcmp(argv[1], "erase") == 0) {
        // config erase
        int res = erase_nvm();
        if (res == 0) {
            printf("Config erased from NVM.\n");
        } else {
            printf("[ERROR] Config erasing failed: %d.\n", res);
        }
    } else {
        return config_help();
    }
    return 0;
}

static char line_buf[SHELL_DEFAULT_BUFSIZE];
static const shell_command_t shell_commands[] =
{
    { "measure", "trigger measure",   measure_cmd },
    { "config",  "configure sensors", config_cmd  },
    { NULL,      NULL,                NULL        },
};

int main(void) {
    ch_group_t *grp_ptr = &soniclib_group;
    uint8_t res;
    uint8_t num_ports;
    uint8_t dev_num;

    printf("\nLoading configuration...");
    if (load_from_nvm(&configuration, sizeof(configuration)) == 0) {
        printf(" found in flash.\n");
    } else {
        config_default();
        printf(" using defaults.\n");
    }
    config_show();

	printf("\nTDK InvenSense\n");
	printf("    Compile time:  %s %s\n", __DATE__, __TIME__);
	printf("    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV);

    res = ch_group_init(grp_ptr, SONICLIB_NUMOF, SONICLIB_NUMOF, SONICLIB_RTC_CAL_PULSE_MS);
    printf("ch_group_init: %d\n", res);

    for (uint8_t i = 0; i < SONICLIB_NUMOF; i++) {
        gpio_init(soniclib_params[i].prog_pin, GPIO_OUT);
        gpio_init(soniclib_params[i].reset_pin, GPIO_OUT);
    }

    printf("Initializing sensor(s)...\n");
    res = 0;
    num_ports = ch_get_num_ports(grp_ptr);
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(soniclib_devices[dev_num]);
        res |= ch_init(dev_ptr, grp_ptr, dev_num, fw_init_func(configuration.firmware));
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

    uint8_t num_connected_sensors = 0;
    uint32_t group_freq = 0;
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
        if (ch_sensor_is_connected(dev_ptr)) {
            num_connected_sensors++;
            uint32_t freq = ch_get_frequency(dev_ptr);
            group_freq += freq;
            printf("%d\tCH%d\t %lu Hz\t%u@%ums\t%s\n", dev_num,
                                    ch_get_part_number(dev_ptr),
                                    freq,
                                    ch_get_rtc_cal_result(dev_ptr),
                                    ch_get_rtc_cal_pulselength(dev_ptr),
                                    ch_get_fw_version_string(dev_ptr));
        }
    }
    printf("\n");
    if (num_connected_sensors)
        group_freq /= num_connected_sensors;

    if (ch_group_set_frequency(grp_ptr, group_freq) == 0) {
        printf("Group nominal frequency: %lu Hz\n", ch_group_get_frequency(grp_ptr));
        printf("After adjustment:\n");
        for (dev_num = 0; dev_num < num_ports; dev_num++) {
            ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
            if (ch_sensor_is_connected(dev_ptr)) {
                printf("%d: %lu Hz\n", dev_num, ch_get_frequency(dev_ptr));
            }
        }
    } else {
        printf("Firmware '%s' does not support setting a group frequency.\n", configuration.firmware);
    }

    // Register callback function for measure ready interrupt
    ch_io_int_callback_set(grp_ptr, sensor_int_callback);

    printf("Configuring sensor(s)...\n");
    apply_configuration();

    printf("Initializing %d x HDC3020\n", HDC3020_NUMOF);
    for (unsigned int i = 0; i < HDC3020_NUMOF; i++) {
        if (hdc3020_init(&hdc3020_devs[i], &hdc3020_params[i]) != HDC3020_OK) {
            hdc3020_data[i].connected = 0;
            printf("Error initializing hdc3020 #%u\n", i);
            continue;
        }
        hdc3020_data[i].connected = 1;
    }

    gpio_init(TRIGGER, GPIO_IN);
    if (gpio_read(TRIGGER) == 0) {
        auto_init_hdc3020();
        printf("Starting shell\n\n");
        shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    } else {
        gpio_init_int(TRIGGER, GPIO_IN, GPIO_FALLING, trigger_callback, NULL);
        printf("Starting measures\n\n");
        while (1) {
            if (taskflags & DATA_READY_FLAG) {
                handle_data_ready(grp_ptr); // fetch available data
                if (configuration.round_robin)
                    apply_configuration();
                taskflags = 0; // now we can start another measure
                print_data(grp_ptr); // print data on console
            } else {
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
        }
    }
    return 0;
}
