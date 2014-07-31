#
# Board-specific definitions for the LEOFMU
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = LEOFMU_V1

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
