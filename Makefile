APPLICATION = anemometer-firmware
BOARD = l21-alone
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0
BAUD ?= 576000
I2C_SPEED ?= FAST_PLUS
FIRMWARE ?= narrow
RANGE ?= 70
ROUNDROBIN ?= 1
PRETRIGGER ?= 0
PRETRIGGER_DELAY ?= 0
ERASE ?=

FIRMWARE_METADATA := \"Compiled at $(shell date +'%Y-%m-%d %H:%M:%S') - git commit $(shell git describe --always)\"
CFLAGS += -DFIRMWARE_METADATA="$(FIRMWARE_METADATA)"

USEMODULE += od
USEMODULE += od_string
USEMODULE += printf_float
USEMODULE += ztimer_msec
USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += shell_extra_commands
USEMODULE += periph_flashpage


USEPKG += soniclib
SONICLIB_FIRMWARE = NONE
SONICLIB_DEBUG_LEVEL = ERROR
CH101_SENSORS := "{.i2c_bus=ACME0_I2C_DEV,.i2c_addr=41,.prog_pin=GPIO_PIN\(PA,10\),.reset_pin=GPIO_PIN\(PA,16\),.io_pin=GPIO_PIN\(PA,1\)},"
CH101_SENSORS += "{.i2c_bus=ACME2_I2C_DEV,.i2c_addr=41,.prog_pin=GPIO_PIN\(PA,7\),.reset_pin=GPIO_PIN\(PA,11\),.io_pin=GPIO_PIN\(PA,0\)}"

I2C_SPEEDS := NORMAL FAST FAST_PLUS
ifeq ($(filter $(I2C_SPEED),$(I2C_SPEEDS)),)
    $(error "$(I2C_SPEED)" not in $(I2C_SPEEDS))
endif
CFLAGS += -DACME0_BUS_MODE=MODE_I2C -DACME0_I2C_SPEED=I2C_SPEED_$(I2C_SPEED)
CFLAGS += -DACME2_BUS_MODE=MODE_I2C -DACME2_I2C_SPEED=I2C_SPEED_$(I2C_SPEED)
CFLAGS += -DSONICLIB_PARAMS="$(CH101_SENSORS)"

USEMODULE += hdc3020
USEMODULE += saul_reg
HDC3020_SENSORS := "{.i2c_dev=ACME0_I2C_DEV,.i2c_addr=HDC3020_PARAM_I2C_ADDR,.alert_pin=HDC3020_ALERT_PIN},"
HDC3020_SENSORS += "{.i2c_dev=ACME2_I2C_DEV,.i2c_addr=HDC3020_PARAM_I2C_ADDR,.alert_pin=HDC3020_ALERT_PIN}"
CFLAGS += -DHDC3020_PARAMS="$(HDC3020_SENSORS)"
CFLAGS += -DHDC3020_SAULINFO="{.name=\"hdc3020-0\"},{.name=\"hdc3020-1\"}"

EXTERNAL_MODULE_DIRS += fw
USEMODULE += ch101

ifneq (, $(RANGE))
  CFLAGS += -DSENSOR_MAX_RANGE_MM=$(RANGE)
endif

ifeq (, $(ROUNDROBIN))
else ifeq (0, $(ROUNDROBIN))
else
  CFLAGS += -DDEFAULT_ROUND_ROBIN=1
endif

ifeq (, $(PRETRIGGER))
else ifeq (0, $(PRETRIGGER))
else
  CFLAGS += -DDEFAULT_RX_PRETRIGGER=1
endif

ifneq (, $(PRETRIGGER_DELAY))
  CFLAGS += -DDEFAULT_RX_PRETRIGGER_DELAY_US=$(PRETRIGGER_DELAY)
endif

ifeq (gpr, $(FIRMWARE))
else ifeq (sr, $(FIRMWARE))
else ifeq (open, $(FIRMWARE))
else ifeq (narrow, $(FIRMWARE))
else
  $(error "Firmware '$(FIRMWARE)' is unsupported.")
endif
CFLAGS += -DDEFAULT_FIRMWARE="\"$(FIRMWARE)\""

ifneq (, $(ERASE))
  EDBG_ARGS += --erase
endif

CFLAGS += -DCLOCK_CORECLOCK=\(48000000U\)
CFLAGS += -DSTDIO_UART_BAUDRATE=$(BAUD)

include $(RIOTBASE)/Makefile.include
