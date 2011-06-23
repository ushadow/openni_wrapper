OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

# Points to the root of Google Test, relative to where this file is.
# Remember to tweak this if you move this file.
GTEST_DIR = /home/yingyin/workspace/gtest-1.6.0

INC_DIRS = $(GTEST_DIR) $(GTEST_DIR)/include ../src \
					 /usr/include/ni /usr/lib/jvm/java-6-sun-1.6.0.24/include \
					 /usr/lib/jvm/java-6-sun-1.6.0.24/include/linux

SRC_FILES = *.cpp ../src/openni_wrapper.cpp $(GTEST_DIR)/src/gtest-all.cc \
						$(GTEST_DIR)/src/gtest_main.cc

EXE_NAME = openni_wrapper_unittest

USED_LIBS += OpenNI

LIB_DIRS += $(GTEST_DIR)/make

CFLAGS = -Wno-deprecated

include ../CommonMakefile
