CUSTOM_CSRCS += $(shell find -L $(PRJ_DIR)/custom -name "*.c")
GEN_CSRCS += $(notdir $(CUSTOM_CSRCS))

DEPPATH += --dep-path $(PRJ_DIR)/custom
VPATH += :$(PRJ_DIR)/custom

CFLAGS += "-I$(PRJ_DIR)/custom"
AFLAGS += "-I$(PRJ_DIR)/custom"
