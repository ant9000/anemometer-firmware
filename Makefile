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
FIRMWARE ?= sr
RANGE ?= 201
ROUNDROBIN ?=

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

CFLAGS += -DACME0_BUS_MODE=MODE_I2C -DACME0_I2C_SPEED=I2C_SPEED_FAST
CFLAGS += -DACME2_BUS_MODE=MODE_I2C -DACME2_I2C_SPEED=I2C_SPEED_FAST
CFLAGS += -DSONICLIB_PARAMS="$(CH101_SENSORS)"

USEMODULE += hdc3020
USEMODULE += saul_reg
HDC3020_SENSORS := "{.i2c_dev=ACME0_I2C_DEV,.i2c_addr=HDC3020_PARAM_I2C_ADDR,.alert_pin=HDC3020_ALERT_PIN},"
HDC3020_SENSORS += "{.i2c_dev=ACME2_I2C_DEV,.i2c_addr=HDC3020_PARAM_I2C_ADDR,.alert_pin=HDC3020_ALERT_PIN}"
CFLAGS += -DHDC3020_PARAMS="$(HDC3020_SENSORS)"
CFLAGS += -DHDC3020_SAULINFO="{.name=\"hdc3020-0\"},{.name=\"hdc3020-1\"}"

EXTERNAL_MODULE_DIRS += fw
USEMODULE += ch101

ifeq (sr, $(FIRMWARE))
  CFLAGS += -DSHORT_RANGE
endif

ifeq (sr_open, $(FIRMWARE))
  CFLAGS += -DSHORT_RANGE_OPEN
endif

ifneq (, $(RANGE))
  CFLAGS += -DSENSOR_MAX_RANGE_MM=$(RANGE)
endif

ifneq (, $(ROUNDROBIN))
  CFLAGS += -DDEFAULT_ROUND_ROBIN=1
endif

CFLAGS += -DCLOCK_CORECLOCK=\(48000000U\)
CFLAGS += -DSTDIO_UART_BAUDRATE=$(BAUD)

include $(RIOTBASE)/Makefile.include
