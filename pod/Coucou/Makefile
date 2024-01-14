# Project Name
TARGET = coucou

# Sources
C_DEFS = -DU8X8_USE_PINS
C_INCLUDES = -IU8g2/src
C_SOURCES =	U8g2/src/clib/u8x8_8x8.c U8g2/src/clib/u8x8_setup.c U8g2/src/clib/u8x8_display.c U8g2/src/clib/u8x8_fonts.c U8g2/src/clib/u8x8_cad.c U8g2/src/clib/u8x8_d_ssd1306_128x64_noname.c U8g2/src/clib/u8x8_gpio.c U8g2/src/clib/u8x8_byte.c
CPP_SOURCES = U8g2/src/U8x8lib.cpp coucou.cpp fft.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

test_fft:
	clang++ test_fft.cpp fft.cpp -lm -o test_fft 
