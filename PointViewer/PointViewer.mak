OSTYPE := $(shell uname -s)
NITE_DIR := /media/WinData/workspace/Tabletop/Nite-1.3.1.5

SRC_FILES = \
	../PointViewer/main.cpp \
	../PointViewer/PointDrawer.cpp \
	../PointViewer/signal_catch.cpp

INC_DIRS += ../PointViewer

EXE_NAME = Sample-PointViewer

ifeq "$(USE_GLES)" "1"
DEFINES = USE_GLES
USED_LIBS += GLES_CM IMGegl srv_um
SRC_FILES += \
	../PointViewer/kbhit.cpp \
	../PointViewer/opengles.cpp
else
DEFINES = USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif
endif
include ../NiteMakefile

