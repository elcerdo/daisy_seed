# Project
TARGET = coucou
# APP_TYPE = BOOT_SRAM

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Flags
OPT = -Os -O2 -Wall -Wextra
# C_DEFS = -DNDEBUG
# LDFLAGS += -u _printf_float

# Sources
CPP_SOURCES = coucou.cpp

# C_DEFS = -DUSE_ARM_FFT

# C_SOURCES += \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal.c\
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c \
# $(LIBDAISY_DIR)/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
# ASM_SOURCES += arm_bitreversal2.s

# CPP_SOURCES += \
# pvoc/stft.cpp \
# pvoc/phase_vocoder.cpp \
# pvoc/frame_transformation.cpp \
# pvoc/resources.cpp


# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

test_fft:
	clang++ test_fft.cpp fft.cpp -lm -o test_fft 
