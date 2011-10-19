OSTYPE := $(shell uname -s)

BIN_DIR = /usr/lib

INC_DIRS = . /usr/include/ni /usr/lib/jvm/java-1.6.0-openjdk/include \
					  /usr/lib/jvm/java-1.6.0-openjdk/include/linux

SRC_FILES = *.cpp

LIB_NAME = openni-java-wrapper

USED_LIBS += OpenNI 

CFLAGS = -Wno-deprecated -std=gnu++0x

include ../CommonMakefile

