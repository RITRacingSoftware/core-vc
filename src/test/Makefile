PROJECT_NAME := vc_test.out

CC := gcc
DEP_FLAGS := -MP -MD

BUILD_DIR := ./build

# Driver
DRIVER_DIR := ../driver
DRIVER_INCS := $(shell find $(DRIVER_DIR) -type d) 
$(info DRV is $(DRIVER_INCS))
DRIVER_INCS := $(addprefix -I, $(DRIVER_INCS))
DRIVER_SRCS := $(shell find $(DRIVER_DIR) -type f -name \*.c)
DRIVER_OBJS := $(DRIVER_SRCS:%=$(BUILD_DIR)/%.o)

# Test
TEST_DIR := ./src
TEST_INCS := -I $(TEST_DIR)
TEST_SRCS := $(shell find $(TEST_DIR) -type f -name \*.c)
TEST_OBJS := $(TEST_SRCS:%=$(BUILD_DIR)%.o)


DBC_INCS := -I ../../../Formula-DBC/c_files
CORE_INCS := -I ../../../Core/src/driver/Inc
RTT_INCS := -I ../../../RTT/RTT


INC_FLAGS := $(CORE_INCS) $(DBC_INCS) $(RTT_INCS) $(TEST_INCS) $(DRIVER_INCS) -I ../
C_FLAGS := $(INC_FLAGS) -D VC_TEST

$(info DBC_INCS is $(DBC_INCS))

OBJS := $(DRIVER_OBJS) $(TEST_OBJS) 

all: $(BUILD_DIR)/$(PROJECT_NAME)

$(BUILD_DIR)/$(PROJECT_NAME) : $(OBJS)
	$(CC) -o $@ $^

$(BUILD_DIR)/%.c.o : %.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) $(C_FLAGS) -c -o $@ $<
	
clean:
	rm -r $(BUILD_DIR)
