CXX = g++
CC  = cc
CXXFLAGS = -Wall -g -std=c++17 -MMD -MP
CFLAGS   = -std=c99 -Wall -Wextra -g
LDFLAGS  =
LDLIBS   = -li2c -lm -lgpiodcxx -lgpiod

OUTDIR   = outputs
BUILDDIR = $(OUTDIR)/build
COBJBASE = $(BUILDDIR)/c

TARGET = $(OUTDIR)/main

# Sensor API locations (relative to pointing/)
MUNIN_DIR     ?= ..
SENSOR_DIR    ?= sensor
BMI_API_DIR   ?= $(SENSOR_DIR)/BMI3XY_SensorAPI
BMI_COMMON    ?= $(BMI_API_DIR)/bmi323_examples/common
BMM_API_DIR   ?= $(SENSOR_DIR)/BMM350_SensorAPI
BMM_COMMON    ?= $(BMM_API_DIR)/examples/common

# C++ source files
CXX_SRC = $(wildcard src/*.cpp)
CXX_OBJ = $(CXX_SRC:src/%.cpp=$(BUILDDIR)/%.o)

# C source files (sensor drivers + Linux I2C transport)
C_SRC = \
    $(BMI_API_DIR)/bmi3.c \
    $(BMI_API_DIR)/bmi323.c \
    $(BMI_COMMON)/common.c \
    $(BMM_API_DIR)/bmm350.c \
    $(BMM_COMMON)/common.c

# Flatten C object paths: all under $(COBJBASE) with hashed names
# so we don't fight with make's path matching on absolute source paths
C_OBJ = $(addprefix $(COBJBASE)/, \
    bmi3.o \
    bmi323.o \
    bmi_common.o \
    bmm350.o \
    bmm_common.o)

# Map object -> source
C_OBJ_SRC = $(COBJBASE)/bmi3.o=$(BMI_API_DIR)/bmi3.c \
            $(COBJBASE)/bmi323.o=$(BMI_API_DIR)/bmi323.c \
            $(COBJBASE)/bmi_common.o=$(BMI_COMMON)/common.c \
            $(COBJBASE)/bmm350.o=$(BMM_API_DIR)/bmm350.c \
            $(COBJBASE)/bmm_common.o=$(BMM_COMMON)/common.c

# Include paths
INCLUDES = \
    -I. \
    -Isrc \
    -I$(SENSOR_DIR) \
    -I$(BMI_API_DIR) \
    -I$(BMI_COMMON) \
    -I$(BMM_API_DIR) \
    -I$(BMM_COMMON)

CXXFLAGS += $(INCLUDES)
CFLAGS   += $(INCLUDES)

# Auto-generated dependency files
DEP = $(CXX_OBJ:.o=.d) $(C_OBJ:.o=.d)

.PHONY: all clean run

all: $(TARGET)

# Link
$(TARGET): $(CXX_OBJ) $(C_OBJ)
	@mkdir -p $(OUTDIR)
	$(CXX) $(CXX_OBJ) $(C_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

# Compile C++ source
$(BUILDDIR)/%.o: src/%.cpp
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile C sources (explicit per-file rules — needed because sources
# live outside the project tree and make's pattern matching won't
# resolve relative-to-cwd paths against an absolute prerequisite)
$(COBJBASE)/bmi3.o: $(BMI_API_DIR)/bmi3.c
	@mkdir -p $(COBJBASE)
	$(CC) $(CFLAGS) -c $< -o $@

$(COBJBASE)/bmi323.o: $(BMI_API_DIR)/bmi323.c
	@mkdir -p $(COBJBASE)
	$(CC) $(CFLAGS) -c $< -o $@

$(COBJBASE)/bmi_common.o: $(BMI_COMMON)/common.c
	@mkdir -p $(COBJBASE)
	$(CC) $(CFLAGS) -c $< -o $@

$(COBJBASE)/bmm350.o: $(BMM_API_DIR)/bmm350.c
	@mkdir -p $(COBJBASE)
	$(CC) $(CFLAGS) -c $< -o $@

$(COBJBASE)/bmm_common.o: $(BMM_COMMON)/common.c
	@mkdir -p $(COBJBASE)
	$(CC) $(CFLAGS) -c $< -o $@

-include $(DEP)

clean:
	rm -rf $(OUTDIR)

run: all
	./$(TARGET)
