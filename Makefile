#############################################################################
# Primesense Experience Engine makefile.
# 
# default configuration is Release. for a debug version use:
# 	make CFG=Debug
#############################################################################
CP = cp
RM = rm
MAKE = make 

# define installation dir
INST_INC = /usr/local/include

# directory containing file to install
FILES_TO_INSTALL_DIR = Install

# rules file to install
#RULES_FILE = 55-primesense-usb.rules

# location for udev rules
INST_RULES_DIR = /etc/udev/rules.d

# default config is Release
ifndef CFG
CFG = Release
endif

# Bin directory
BIN_DIR = ../Bin
# output directory
OUT_DIR = $(BIN_DIR)

# include directory (for installing)
INC_DIR = ../../../Include
# create a list of all include files under the directory
INC_FILES = $(wildcard $(INC_DIR)/*.h)
# names of additional directories under the include directory which should be copied
INC_DIRS = Linux-x86

# list all projects (in the order they should compile)
ALL_PROJS = PointViewer 

# define a function which creates a make command for every makefile ($1 = additional params)
MAKE_ALL_MAKEFILES = $(foreach proj,$(ALL_PROJS),$(MAKE) -C $(proj) CFG=$(CFG) -f $(notdir $(proj)).mak $1;)

.PHONY: all install install-projs install-includes install-usb-rules uninstall uninstall-projs uninstall-includes uninstall-usb-rules clean

# make all makefiles
all:
	$(call MAKE_ALL_MAKEFILES)

# install target
install: install-projs install-includes install-usb-rules

# install each project
install-projs:
	$(call MAKE_ALL_MAKEFILES,install)

# install include files
install-includes:
	$(CP) $(INC_FILES) $(INST_INC)
	$(CP) -r $(addprefix $(INC_DIR)/,$(INC_DIRS)) $(INST_INC)

# install usb mount rules (if not installed, usb device will not have writing permissions)
install-usb-rules: $(FILES_TO_INSTALL_DIR)/$(RULES_FILE)
	$(CP) $(FILES_TO_INSTALL_DIR)/$(RULES_FILE) $(INST_RULES_DIR)

# uninstall target
uninstall: uninstall-projs uninstall-includes uninstall-usb-rules

# uninstall each project
uninstall-projs:
	$(call MAKE_ALL_MAKEFILES,uninstall)

# uninstall include files
uninstall-includes:
	$(RM) -f $(addprefix $(INST_INC)/,$(notdir $(INC_FILES)))
	$(RM) -f -r $(addprefix $(INST_INC)/,$(INC_DIRS))

# uninstall usb mount rules
uninstall-usb-rules:
	$(RM) -f $(INST_RULES_DIR)/$(RULES_FILE)

# clean is cleaning all projects
clean: 
	$(call MAKE_ALL_MAKEFILES,clean)

