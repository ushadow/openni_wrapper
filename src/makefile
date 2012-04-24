OSTYPE := $(shell uname -s)

BIN_DIR = /usr/lib

INC_DIRS = . /usr/include/ni /usr/lib/jvm/java-6-sun/include \
					  /usr/lib/jvm/java-6-sun/include/linux

SRC_FILES = *.cpp

LIB_NAME = openni-java-wrapper

USED_LIBS += OpenNI 

CFLAGS = -Wno-deprecated -std=gnu++0x

include ../CommonMakefile

