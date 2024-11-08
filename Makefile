APPLICATION = anemometer-firmware
BOARD = berta-h10
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += od
USEMODULE += od_string
USEMODULE += printf_float
USEMODULE += periph_spi_reconfigure
USEMODULE += ztimer_msec
USEMODULE += saul_default

USEPKG += soniclib
SONICLIB_FIRMWARE = CH101_GPR
SONICLIB_DEBUG_LEVEL = INFO
SENSORS := "{.i2c_bus=ACME1_I2C_DEV,.i2c_addr=41,.prog_pin=GPIO_PIN\(PB,23\),.io_pin=GPIO_PIN\(PA,1\)},"
SENSORS += "{.i2c_bus=ACME2_I2C_DEV,.i2c_addr=41,.prog_pin=GPIO_PIN\(PA,7\),.io_pin=GPIO_PIN\(PA,0\)}"

CFLAGS += -DACME1_BUS_MODE=MODE_I2C
CFLAGS += -DACME2_BUS_MODE=MODE_I2C
CFLAGS += -DSONICLIB_PARAMS="$(SENSORS)"

include $(RIOTBASE)/Makefile.include
