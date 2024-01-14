# Project Name
TARGET = coucou

# Sources
C_DEFS =
C_INCLUDES =
C_SOURCES =	
CPP_SOURCES = coucou.cpp fft.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

test_fft:
	clang++ test_fft.cpp fft.cpp -lm -o test_fft 
