#
# Makefile for the leofmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common
# ROMFS_OPTIONAL_FILES = $(PX4_BASE)/Images/px4io-v1_default.bin

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
#MODULES		+= drivers/stm32/tone_alarm
MODULES		+= drivers/led
#MODULES		+= drivers/px4io
MODULES		+= drivers/px4fmu
MODULES		+= drivers/boards/leofmu-v1
MODULES 	+= drivers/lsm9ds0
MODULES 	+= drivers/lsm9ds0_gyro
MODULES 	+= drivers/lps331
# MODULES		+= drivers/ardrone_interface
# MODULES		+= drivers/l3gd20
# MODULES		+= drivers/mpu6000
# MODULES		+= drivers/hmc5883
# MODULES		+= drivers/ms5611
# MODULES		+= drivers/mb12xx
MODULES		+= drivers/gps
MODULES		+= drivers/hil
#MODULES		+= drivers/hott/hott_telemetry
#MODULES		+= drivers/hott/hott_sensors
MODULES		+= drivers/blinkm
MODULES		+= drivers/rgbled
MODULES		+= drivers/mkblctrl
MODULES		+= drivers/airspeed
MODULES		+= drivers/ets_airspeed
MODULES		+= drivers/meas_airspeed
MODULES		+= drivers/frsky_telemetry
MODULES		+= modules/sensors

#
# System commands
#
MODULES		+= systemcmds/mtd
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/i2c
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/param
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/preflight_check
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/esc_calib
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
#MODULES		+= systemcmds/tests
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm
MODULES		+= systemcmds/dumpfile
MODULES		+= systemcmds/ver

#
# General system control
#
MODULES		+= modules/commander
MODULES		+= modules/navigator
MODULES		+= modules/mavlink
MODULES		+= modules/gpio_led

#
# Estimation modules (EKF/ SO3 / other filters)
#
MODULES		+= modules/attitude_estimator_ekf
MODULES		+= modules/ekf_att_pos_estimator
MODULES		+= modules/position_estimator_inav
#MODULES		+= examples/flow_position_estimator

#
# Vehicle Control
#
MODULES		+= modules/fw_pos_control_l1
MODULES		+= modules/fw_att_control
MODULES		+= modules/mc_att_control
MODULES		+= modules/mc_pos_control
#MODULES		+= examples/flow_position_control
#MODULES		+= examples/flow_speed_control

#
# Logging
#
MODULES		+= modules/sdlog2

#
# Unit tests
#
#MODULES 	+= modules/unit_test
#MODULES 	+= modules/commander/commander_tests

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/controllib
MODULES		+= modules/uORB
MODULES		+= modules/dataman

#
# Libraries
#
LIBRARIES	+= lib/mathlib/CMSIS
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/ecl
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
MODULES		+= lib/launchdetection

#
# Demo apps
#
#MODULES		+= examples/math_demo
# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/hello_sky
#MODULES		+= examples/px4_simple_app

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/daemon
#MODULES		+= examples/px4_daemon_app

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/debug_values
#MODULES		+= examples/px4_mavlink_debug

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/example_fixedwing_control
#MODULES			+= examples/fixedwing_control

# Hardware test
#MODULES			+= examples/hwtest

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                ) \
	$(call _B, sysinfo,                ,                          2048,  sysinfo_main               )
