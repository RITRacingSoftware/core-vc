PROJECT_NAME := vc_test.out

CC := gcc
INC_FLAGS := $(addprefix -I, $(DRIVER_INCS)$(TEST_INCS).)
DEP_FLAGS := -MP -MD
C_FLAGS := $(INC_FLAGS) $(DEP_FLAGS) -DTEST
# C_FLAGS :=  $(DEP_FLAGS) -DTEST

BUILD_DIR := ./build
TEST_DIR := ./src
DRIVER_DIR := ../driver

INCS := $(TEST_DIR) $(DRIVER_DIR) .

# DRIVER_INCS := $(shell find $(DRIVER_DIR) -type d) 
# TEST_INCS := $(shell find $(TEST_DIR) -type d) 


TEST_SRCS := $(shell find $(TEST_DIR) -type f -name \*.c)
DRIVER_SRCS := $(shell find $(DRIVER_DIR) -type f -name \*.c)
C_FILES := $(TEST_SRCS) $(DRIVER_SRCS) ./main.c

OBJS := $(C_FILES:%=$(BUILD_DIR)/%.o) 

TEST_DIRS := $(shell find $(TEST_DIR) -type d)
DRIVER_DIRS := $(shell find $(DRIVER_DIR) -type d)

# INC_FLAGS := $(addprefix -I, $(TEST_DIRS)$(DRIVER_DIRS))
INC_FLAGS := $(addprefix -I, ../) 

$(info DRIVER_DIRS is $(DRIVER_DIRS))
$(info TEST_DIRS is $(TEST_DIRS))



# TEST_OBJS := $(patsubst %.c, %.o, $(shell basename $(foreach d, $(TEST_SRCS), $(d))))
# DRIVER_OBJS := $(patsubst %.c, %.o, $(shell basename $(foreach d, $(DRIVER_SRCS), $(d))))
# OBJS = $(TEST_OBJS) $(DRIVER_OBJS) main.o
# OBJS := $(foreach d, $(OBJS), $(BUILD_DIR)/$(d))
# DEP_FILES := $(patsubst %.o, %.d, $(OBJS))

# $(DRIVER_SRCS:$(DRIVER_DIR)/% = $(BUILD_DIR)/driver/%.o) 
$(info OBJS is $(OBJS))
$(info SRCS is $(C_FILES))

# DEPS := $(patsubst %.c, %.d, $(C_FILES))
# $(info OBJS is $(OBJS))
# $(info SRCS is $(C_FILES))
# Build final executable

all: $(BUILD_DIR)/$(PROJECT_NAME)

$(BUILD_DIR)/$(PROJECT_NAME) : $(OBJS)
	$(CC) -o  $@ $^

$(BUILD_DIR)/%.c.o : %.c
	@[ -d $(@D) ] || mkdir -p $(@D)
	$(CC) -DTEST $(INC_FLAGS) -c -o $@ $<
	
clean:
	rm -r $(BUILD_DIR)
