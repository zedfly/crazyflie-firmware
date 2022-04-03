
<<<<<<< HEAD
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_CMDS      ?=

# Cload is handled in a special way on windows in WSL to use the Windows python interpreter
ifdef WSL_DISTRO_NAME
PYTHON      := python.exe
else
PYTHON      ?= python3
endif

DFU_UTIL          ?= dfu-util

CLOAD_SCRIPT      ?= $(PYTHON) -m cfloader
CLOAD_CMDS        ?=
CLOAD_ARGS        ?=

ARCH := stm32f4
SRCARCH := stm32f4
=======
CRAZYFLIE_BASE ?= ./

# Put your personal build config in tools/make/config.mk and DO NOT COMMIT IT!
# Make a copy of tools/make/config.mk.example to get you started
-include tools/make/config.mk

CFLAGS += $(EXTRA_CFLAGS)
CXXFLAGS += $(EXTRA_CFLAGS)

######### JTAG and environment configuration ##########
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_CMDS      ?=
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON            ?= python3
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= python3 -m cfloader
CLOAD_CMDS        ?=
CLOAD_ARGS        ?=
PLATFORM          ?= cf2
LPS_TDMA_ENABLE   ?= 0
LPS_TDOA_ENABLE   ?= 0
LPS_TDOA3_ENABLE  ?= 0


# Platform configuration handling
-include current_platform.mk
include $(CRAZYFLIE_BASE)/tools/make/platform.mk

CFLAGS += -DCRAZYFLIE_FW
CXXFLAGS += -DCRAZYFLIE_FW

#OpenOCD conf
RTOS_DEBUG        ?= 0

LIB = $(CRAZYFLIE_BASE)/src/lib
FREERTOS = $(CRAZYFLIE_BASE)/vendor/FreeRTOS
CFLAGS += -DBLOBS_LOC='"$(CRAZYFLIE_BASE)/blobs/"'
CXXFLAGS += -DBLOBS_LOCK='"$(CRAZYFLIE_BASE)/blobs/"'

# Communication Link
UART2_LINK        ?= 0


############### CPU-specific build configuration ################

ifeq ($(CPU), stm32f4)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
LINKER_DIR = $(CRAZYFLIE_BASE)/tools/make/F405/linker
ST_OBJ_DIR  = $(CRAZYFLIE_BASE)/tools/make/F405

OPENOCD_TARGET    ?= target/stm32f4x.cfg


# St Lib
VPATH += $(LIB)/CMSIS/STM32F4xx/Source/
VPATH += $(LIB)/STM32_USB_Device_Library/Core/src
VPATH += $(LIB)/STM32_USB_OTG_Driver/src
VPATH += $(CRAZYFLIE_BASE)/src/deck/api $(CRAZYFLIE_BASE)/src/deck/core $(CRAZYFLIE_BASE)/src/deck/drivers/src $(CRAZYFLIE_BASE)/src/deck/drivers/src/test
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/tdoa $(CRAZYFLIE_BASE)/src/utils/src/lighthouse
CRT0 = startup_stm32f40xx.o system_stm32f4xx.o

# Add ST lib object files
-include $(ST_OBJ_DIR)/st_obj.mk

# USB obj
ST_OBJ += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ += usbd_ioreq.o usbd_req.o usbd_core.o

PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee
CXXFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee

#Flags required by the ST library
CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER
CXXFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

LOAD_ADDRESS_stm32f4 = 0x8000000
LOAD_ADDRESS_CLOAD_stm32f4 = 0x8004000
MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64
endif

################ Build configuration ##################

# libdw dw1000 driver
VPATH += $(CRAZYFLIE_BASE)/vendor/libdw1000/src

# vl53l1 driver
VPATH += $(LIB)/vl53l1/core/src

# FreeRTOS
VPATH += $(PORT)
PORT_OBJ = port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ ?= heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o event_groups.o $(MEMMANG_OBJ)

#FatFS
VPATH += $(LIB)/FatFS
PROJ_OBJ += ff.o ffunicode.o fatfs_sd.o
ifeq ($(FATFS_DISKIO_TESTS), 1)
PROJ_OBJ += diskio_function_tests.o
CFLAGS += -DUSD_RUN_DISKIO_FUNCTION_TESTS
CXXFLAGS += -DUSD_RUN_DISKIO_FUNCTION_TESTS
endif

ifeq ($(APP), 1)
CFLAGS += -DAPP_ENABLED=1
CXXFLAGS += -DAPP_ENABLED=1
endif
ifdef APP_STACKSIZE
CFLAGS += -DAPP_STACKSIZE=$(APP_STACKSIZE)
CXXFLAGS += -DAPP_STACKSIZE=$(APP_STACKSIZE)
endif
ifdef APP_PRIORITY
CFLAGS += -DAPP_PRIORITY=$(APP_PRIORITY)
CXXFLAGS += -DAPP_PRIORITY=$(APP_PRIORITY)
endif

# Crazyflie sources
VPATH += $(CRAZYFLIE_BASE)/src/init $(CRAZYFLIE_BASE)/src/hal/src $(CRAZYFLIE_BASE)/src/modules/src $(CRAZYFLIE_BASE)/src/modules/src/lighthouse $(CRAZYFLIE_BASE)/src/modules/src/kalman_core $(CRAZYFLIE_BASE)/src/utils/src $(CRAZYFLIE_BASE)/src/drivers/bosch/src $(CRAZYFLIE_BASE)/src/drivers/src $(CRAZYFLIE_BASE)/src/platform $(CRAZYFLIE_BASE)/src/drivers/esp32/src
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/kve

############### Source files configuration ################

# Init
PROJ_OBJ += main.o
PROJ_OBJ += platform.o platform_utils.o platform_$(PLATFORM).o platform_$(CPU).o

# Drivers
PROJ_OBJ += exti.o nvic.o motors.o
PROJ_OBJ += led.o mpu6500.o i2cdev.o ws2812_cf2.o lps25h.o i2c_drv.o
PROJ_OBJ += ak8963.o eeprom.o maxsonar.o piezo.o
PROJ_OBJ += uart_syslink.o swd.o uart1.o uart2.o watchdog.o
PROJ_OBJ += cppm.o
PROJ_OBJ += bmi055_accel.o bmi055_gyro.o bmi160.o bmp280.o bstdr_comm_support.o bmm150.o
PROJ_OBJ += bmi088_accel.o bmi088_gyro.o bmi088_fifo.o bmp3.o
PROJ_OBJ += pca9685.o vl53l0x.o pca95x4.o pca9555.o vl53l1x.o pmw3901.o
PROJ_OBJ += amg8833.o lh_bootloader.o
PROJ_OBJ += esp_rom_bootloader.o esp_slip.o

# USB Files
PROJ_OBJ += usb_bsp.o usblink.o usbd_desc.o usb.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o buzzer.o
PROJ_OBJ += pm_$(CPU).o syslink.o radiolink.o ow_syslink.o ow_common.o proximity.o usec_time.o
PROJ_OBJ += sensors.o
PROJ_OBJ += storage.o

# libdw
PROJ_OBJ += libdw1000.o libdw1000Spi.o

# vl53l1 lib
PROJ_OBJ += vl53l1_api_core.o vl53l1_api.o vl53l1_core.o vl53l1_silicon_core.o vl53l1_api_strings.o
PROJ_OBJ += vl53l1_api_calibration.o vl53l1_api_debug.o vl53l1_api_preset_modes.o vl53l1_error_strings.o
PROJ_OBJ += vl53l1_register_funcs.o vl53l1_wait.o vl53l1_core_support.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param_task.o param_logic.o
PROJ_OBJ += log.o worker.o queuemonitor.o msp.o
PROJ_OBJ += platformservice.o sound_cf2.o extrx.o sysload.o mem.o
PROJ_OBJ += range.o app_handler.o static_mem.o app_channel.o
PROJ_OBJ += eventtrigger.o supervisor.o

# Stabilizer modules
PROJ_OBJ += commander.o crtp_commander.o crtp_commander_rpyt.o
PROJ_OBJ += crtp_commander_generic.o crtp_localization_service.o peer_localization.o
PROJ_OBJ += attitude_pid_controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += position_estimator_altitude.o position_controller_pid.o position_controller_indi.o
PROJ_OBJ += estimator.o estimator_complementary.o
PROJ_OBJ += controller.o controller_pid.o controller_mellinger.o controller_indi.o
PROJ_OBJ += power_distribution_$(POWER_DISTRIBUTION).o
PROJ_OBJ += collision_avoidance.o health.o

# Kalman estimator
PROJ_OBJ += estimator_kalman.o kalman_core.o kalman_supervisor.o
PROJ_OBJ += mm_distance.o mm_absolute_height.o mm_position.o mm_pose.o mm_tdoa.o mm_flow.o mm_tof.o mm_yaw_error.o mm_sweep_angles.o
PROJ_OBJ += mm_tdoa_robust.o mm_distance_robust.o

# High-Level Commander
PROJ_OBJ += crtp_commander_high_level.o planner.o pptraj.o pptraj_compressed.o

# Deck Core
PROJ_OBJ += deck.o deck_info.o deck_drivers.o deck_test.o deck_memory.o

# Deck API
PROJ_OBJ += deck_constants.o
PROJ_OBJ += deck_digital.o
PROJ_OBJ += deck_analog.o
PROJ_OBJ += deck_spi.o
PROJ_OBJ += deck_spi3.o

# Decks
PROJ_OBJ += bigquad.o
PROJ_OBJ += ledring12.o
PROJ_OBJ += buzzdeck.o
PROJ_OBJ += gtgps.o
PROJ_OBJ += cppmdeck.o
PROJ_OBJ += usddeck.o
PROJ_OBJ += zranger.o zranger2.o
PROJ_OBJ += locodeck.o
PROJ_OBJ += clockCorrectionEngine.o
PROJ_OBJ += lpsTwrTag.o
PROJ_OBJ += lpsTdoa2Tag.o
PROJ_OBJ += lpsTdoa3Tag.o tdoaEngineInstance.o tdoaEngine.o tdoaStats.o tdoaStorage.o
PROJ_OBJ += outlierFilter.o
PROJ_OBJ += flowdeck_v1v2.o
PROJ_OBJ += oa.o
PROJ_OBJ += multiranger.o
PROJ_OBJ += lighthouse.o
PROJ_OBJ += activeMarkerDeck.o

# Uart2 Link for CRTP communication is not compatible with decks using uart2
ifeq ($(UART2_LINK), 1)
CFLAGS += -DUART2_LINK_COMM
CXXFLAGS += -DUART2_LINK_COMM
else
PROJ_OBJ += aideck.o
endif

ifeq ($(LPS_TDOA_ENABLE), 1)
CFLAGS += -DLPS_TDOA_ENABLE
CXXFLAGS += -DLPS_TDOA_ENABLE
endif

ifeq ($(LPS_TDOA3_ENABLE), 1)
CFLAGS += -DLPS_TDOA3_ENABLE
CXXFLAGS += -DLPS_TDOA3_ENABLE
endif

ifeq ($(LPS_TDMA_ENABLE), 1)
CFLAGS += -DLPS_TDMA_ENABLE
CXXFLAGS += -DLPS_TDMA_ENABLE
endif

ifdef SENSORS
SENSORS_UPPER = $(shell echo $(SENSORS) | tr a-z A-Z)
CFLAGS += -DSENSORS_FORCE=SensorImplementation_$(SENSORS)
CXXFLAGS += -DSENSORS_FORCE=SensorIMplementation_$(SENSORS)

# Add sensor file to the build if needed
ifeq (,$(findstring DSENSOR_INCLUDED_$(SENSORS_UPPER),$(CFLAGS)))
CFLAGS += -DSENSOR_INCLUDED_$(SENSORS_UPPER)
CXXFLAGS += -DSENSOR_INCLUDED_$(SENSORS_UPPER)
PROJ_OBJ += sensors_$(SENSORS).o
endif
endif

#Deck tests
PROJ_OBJ += exptest.o
PROJ_OBJ += exptestRR.o
PROJ_OBJ += exptestBolt.o
#PROJ_OBJ += bigquadtest.o
#PROJ_OBJ += uarttest.o
#PROJ_OBJ += aidecktest.o


# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o  eprintf.o crc32.o num.o debug.o
PROJ_OBJ += version.o FreeRTOS-openocd.o
PROJ_OBJ += configblockeeprom.o
PROJ_OBJ += sleepus.o statsCnt.o rateSupervisor.o
PROJ_OBJ += lighthouse_core.o pulse_processor.o pulse_processor_v1.o pulse_processor_v2.o lighthouse_geometry.o ootx_decoder.o lighthouse_calibration.o lighthouse_deck_flasher.o lighthouse_position_est.o lighthouse_storage.o lighthouse_transmit.o
PROJ_OBJ += kve_storage.o kve.o
PROJ_OBJ += esp_deck_flasher.o

ifeq ($(DEBUG_PRINT_ON_SEGGER_RTT), 1)
VPATH += $(LIB)/Segger_RTT/RTT
INCLUDES += -I$(LIB)/Segger_RTT/RTT
PROJ_OBJ += SEGGER_RTT.o SEGGER_RTT_printf.o
CFLAGS += -DDEBUG_PRINT_ON_SEGGER_RTT
CXXFLAGS += -DDEBUG_PRINT_ON_SEGGER_RTT
endif
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51

ARCH_CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3
ARCH_CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee
ARCH_CFLAGS += -Wno-address-of-packed-member -Wno-array-bounds -Wno-stringop-overread
ARCH_CFLAGS += -Wno-stringop-overflow
ARCH_CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

FREERTOS = $(srctree)/vendor/FreeRTOS
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
LIB = $(srctree)/src/lib
PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LINKER_DIR = $(srctree)/tools/make/F405/linker

<<<<<<< HEAD
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -nostdlib
image_LDFLAGS += -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority
image_LDFLAGS += -L$(srctree)/tools/make/F405/linker
image_LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld
=======
############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)g++
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51

INCLUDES += -I$(srctree)/vendor/CMSIS/CMSIS/Core/Include -I$(srctree)/vendor/CMSIS/CMSIS/DSP/Include
INCLUDES += -I$(srctree)/vendor/libdw1000/inc
INCLUDES += -I$(FREERTOS)/include -I$(PORT)
INCLUDES += -I$(srctree)/src/config
INCLUDES += -I$(srctree)/src/platform/interface
INCLUDES += -I$(srctree)/src/deck/interface -I$(srctree)/src/deck/drivers/interface
INCLUDES += -I$(srctree)/src/drivers/interface -I$(srctree)/src/drivers/bosch/interface
INCLUDES += -I$(srctree)/src/drivers/esp32/interface
INCLUDES += -I$(srctree)/src/hal/interface
INCLUDES += -I$(srctree)/src/modules/interface -I$(srctree)/src/modules/interface/kalman_core -I$(srctree)/src/modules/interface/lighthouse
INCLUDES += -I$(srctree)/src/utils/interface -I$(srctree)/src/utils/interface/kve -I$(srctree)/src/utils/interface/lighthouse -I$(srctree)/src/utils/interface/tdoa
INCLUDES += -I$(LIB)/FatFS
INCLUDES += -I$(LIB)/CMSIS/STM32F4xx/Include
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(LIB)/vl53l1 -I$(LIB)/vl53l1/core/inc
INCLUDES += -I$(KBUILD_OUTPUT)/include/generated

<<<<<<< HEAD
# Here we tell Kbuild where to look for Kbuild files which will tell the
# buildsystem which sources to build
objs-y += src
objs-y += vendor

objs-y += app_api
objs-y += $(OOT)

MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64
=======
CFLAGS += -g3
CXXFLAGS += -g3
ifeq ($(DEBUG), 1)
  CFLAGS += -Og -DDEBUG
	CXXFLAGS += -Og -DDEBUG

  # Prevent silent errors when converting between types (requires explicit casting)
  CFLAGS += -Wconversion
	CXXFLAGS += -Wconversion
else
  CFLAGS += -Os

  # Fail on warnings
  CFLAGS += -Werror
	CXXFLAGS += -Werror
endif
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51

KBUILD_OUTPUT ?= build

-include $(KBUILD_OUTPUT)/include/config/auto.conf

<<<<<<< HEAD
#
# Special hack to handle float define. Kconfig has no float values
# so we use string. To avoid having to do atof in code we instead
# catch it here and convert to an, unquoted, float define.
#
ifneq ($(CONFIG_DECK_LOCO_2D_POSITION_HEIGHT),)
unquoted = $(patsubst "%",%,$(CONFIG_DECK_LOCO_2D_POSITION_HEIGHT))
ARCH_CFLAGS += -DDECK_LOCO_2D_POSITION_HEIGHT=$(unquoted)
endif

ifeq ($(CONFIG_PLATFORM_TAG),y)
PLATFORM = tag
=======
ifeq ($(LTO), 1)
  CFLAGS += -flto
	CXXFLAGS += -flto
endif

CFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)
CXXFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)

CFLAGS += $(PROCESSOR) $(INCLUDES)
CXXFLAGS += $(PROCESSOR) $(INCLUDES)

CXXFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(INCLUDES)

CXXFLAGS += -std=c++17 -Werror

CFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11
CXXFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing
# CFLAGS += -O0 -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11 #Use this compiler during debugger, as it has a different optimizer so you can better track variables
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(@).d -MQ $(@)
#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority
LDFLAGS += -L$(CRAZYFLIE_BASE)/tools/make/F405/linker

#Flags required by the ST library
ifeq ($(CLOAD), 1)
  LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_CLOAD_$(CPU))
else
  LDFLAGS += -T $(LINKER_DIR)/FLASH.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_$(CPU))
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51
endif

ifeq ($(CONFIG_PLATFORM_BOLT), y)
PLATFORM = bolt
endif

<<<<<<< HEAD
PLATFORM  ?= cf2
=======
CXXFLAGS += -fuse-ld=gold

#Program name
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51
PROG ?= $(PLATFORM)

ifeq ($(CONFIG_DEBUG),y)
ARCH_CFLAGS	+= -O0 -Wconversion
else
ARCH_CFLAGS += -Os -Werror
endif

<<<<<<< HEAD
_all:
=======
#################### Targets ###############################

all: bin/ bin/dep bin/vendor build
build:
# Each target is in a different line, so they are executed one after the other even when the processor has multiple cores (when the -j option for the make command is > 1). See: https://www.gnu.org/software/make/manual/html_node/Parallel.html
	@$(MAKE) --no-print-directory clean_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory compile CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory print_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory size CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
compile: $(PROG).hex $(PROG).bin $(PROG).dfu

bin/:
	mkdir -p bin
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51

all: $(PROG).hex $(PROG).bin
	@echo "Build for the $(PLATFORM)!"
	@$(PYTHON) $(srctree)/tools/make/versionTemplate.py --crazyflie-base $(srctree) --print-version
	@$(PYTHON) $(srctree)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

	#
	# Create symlinks to the ouput files in the build directory
	#
	for f in $$(ls $(PROG).*); do \
		ln -sf $(KBUILD_OUTPUT)/$$f $(srctree)/$$(basename $$f); \
	done

oot-config:
	[ ! -e "$(OOT_CONFIG)" ] || $(srctree)/scripts/kconfig/merge_config.sh $(OOT_CONFIG)

include tools/make/targets.mk

size:
	@$(PYTHON) $(srctree)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

# Radio bootloader
CLOAD ?= 1
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) $(CLOAD_CMDS) flash $(CLOAD_ARGS) $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

unit:
# The flag "-DUNITY_INCLUDE_DOUBLE" allows comparison of double values in Unity. See: https://stackoverflow.com/a/37790196
	rake unit "DEFINES=$(ARCH_CFLAGS) -DUNITY_INCLUDE_DOUBLE" "FILES=$(FILES)" "UNIT_TEST_STYLE=$(UNIT_TEST_STYLE)"

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#verify only
flash_verify:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

flash_dfu:
	$(DFU_UTIL) -a 0 -D $(PROG).dfu

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$$_TARGETNAME configure -rtos auto"

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

rtt:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets \
	           -c "rtt setup 0x20000000 262144 \"SEGGER RTT\"" -c "rtt start" -c "rtt server start 2000 0"

gdb: $(PROG).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

#Print preprocessor #defines
prep:
	@$(CC) $(CFLAGS) -dM -E - < /dev/null

<<<<<<< HEAD
check_submodules:
	@cd $(srctree); $(PYTHON) tools/make/check-for-submodules.py

# Give control over to Kbuild
-include tools/kbuild/Makefile.kbuild
=======
include $(CRAZYFLIE_BASE)/tools/make/targets.mk
>>>>>>> 9e25d15d195d265243a102d35a9c0fb7b289bc51


ifeq ($(KBUILD_SRC),)
# Python bindings
MOD_INC = src/modules/interface
MOD_SRC = src/modules/src

bindings_python: bindings/setup.py bin/cffirmware_wrap.c $(MOD_SRC)/*.c
	$(PYTHON) bindings/setup.py build_ext --inplace

bin/cffirmware_wrap.c cffirmware.py: bindings/cffirmware.i $(MOD_INC)/*.h
	swig -python -I$(MOD_INC) -o bin/cffirmware_wrap.c bindings/cffirmware.i
	mv bin/cffirmware.py cffirmware.py

test_python: bindings_python
	$(PYTHON) -m pytest test_python
endif

.PHONY: all clean build compile unit prep erase flash trace openocd gdb halt reset flash_dfu flash_verify cload size clean_version bindings_python
