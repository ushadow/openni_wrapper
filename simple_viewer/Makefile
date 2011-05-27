OSTYPE := $(shell uname -s)

BIN_DIR = ../bin

INC_DIRS = /usr/include/ni /usr/include/pcl-1.0 /usr/include/eigen3 \
					 /usr/include/vtk-5.4

SRC_FILES = *.cpp

EXE_NAME = simple_viewer

USED_LIBS += OpenNI pcl_common pcl_visualization

CFG = Debug
CFLAGS = -Wno-deprecated

include ../CommonMakefile

