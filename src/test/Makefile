PROJECT_NAME := vc_test.out

CC := gcc
DEP_FLAGS := -MP -MD

BUILD_DIR := ./build

# Driver
DRIVER_DIR := ../driver
DRIVER_BUILD_DIR := $(BUILD_DIR)/driver
DRIVER_INCS := $(shell find $(DRIVER_DIR) -type d) 
DRIVER_INCS := $(addprefix -I, $(DRIVER_INCS))
DRIVER_SRCS := $(shell find $(DRIVER_DIR) -type f -name \*.c)
DRIVER_OBJS := $(DRIVER_SRCS:$(DRIVER_DIR)/%=$(DRIVER_BUILD_DIR)/%.o)

# Test
TEST_DIR := ./src
TEST_BUILD_DIR := $(BUILD_DIR)/test
TEST_INCS := -I $(TEST_DIR) -I $(TEST_DIR)/core -I $(TEST_DIR)/tests
TEST_SRCS := $(shell find $(TEST_DIR) -type f -name \*.c)
TEST_OBJS := $(TEST_SRCS:$(TEST_DIR)/%=$(TEST_BUILD_DIR)/%.o)

# FREERTOS
FREERTOS_DIR := ../../../FreeRTOS-Kernel
FREERTOS_INCS := $(FREERTOS_DIR)/include $(FREERTOS_DIR)/portable/GCC/ARM_CM4F
FREERTOS_INCS := $(foreach d, $(FREERTOS_INCS),-I $d)

# Core
CORE_DIR := ../../../Core/src/mock
CORE_BUILD_DIR := $(BUILD_DIR)/core
CORE_INCS := -I $(CORE_DIR)/Inc
CORE_SRCS := $(shell find $(CORE_DIR) -type f -name \*.c)
CORE_OBJS := $(CORE_SRCS:$(CORE_DIR)/%=$(CORE_BUILD_DIR)/%.o)

# DBC
DBC_DIR := ../../../Formula-DBC/c_files
DBC_BUILD_DIR := $(BUILD_DIR)/dbc
DBC_INCS := -I $(DBC_DIR)
DBC_SRCS := $(shell find $(DBC_DIR) -type f -name \*.c)
DBC_OBJS := $(DBC_SRCS:$(DBC_DIR)/%=$(DBC_BUILD_DIR)/%.o)

HAL_INCS := -I ../../../Core/src/mock


INC_FLAGS := -I ../ $(CORE_INCS) $(DBC_INCS) $(HAL_INCS) $(FREERTOS_INCS)  $(DRIVER_INCS) $(TEST_INCS)
C_FLAGS := $(INC_FLAGS) -D VC_TEST -lm

OBJS := $(CORE_OBJS) $(DRIVER_OBJS) $(TEST_OBJS) $(DBC_OBJS)


all: $(BUILD_DIR)/$(PROJECT_NAME)

$(BUILD_DIR)/$(PROJECT_NAME) : $(OBJS) 
	$(CC) -o $@ $^


# Driver objs
$(DRIVER_BUILD_DIR)/%.c.o : $(DRIVER_DIR)/%.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) $(C_FLAGS) -c -o $@ $<


# Test objs
$(TEST_BUILD_DIR)/%.c.o : $(TEST_DIR)/%.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) $(C_FLAGS) -c -o $@ $<


# Core objs
$(CORE_BUILD_DIR)/%.c.o : $(CORE_DIR)/%.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) $(C_FLAGS) -c -o $@ $<


# DBC objs
$(DBC_BUILD_DIR)/%.c.o : $(DBC_DIR)/%.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) $(C_FLAGS) -c -o $@ $<


clean:
	rm -r $(BUILD_DIR)
