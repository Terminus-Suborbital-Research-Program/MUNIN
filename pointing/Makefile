CXX = g++
CC = gcc

CXXFLAGS = -Wall -g -Iinclude -MMD -MP
CFLAGS   = -Wall -g -Iinclude -MMD -MP

LDFLAGS = -lgpiodcxx -li2c

OUTDIR   = outputs
BUILDDIR = $(OUTDIR)/build
TARGET   = $(OUTDIR)/main

# C++ source files
CPP_SRC := $(shell find src -type f -name '*.cpp')

# Bosch sensor driver source files
C_SRC := \
	sensor/BMI3XY_SensorAPI/bmi3.c \
	sensor/BMI3XY_SensorAPI/bmi323.c \
    sensor/BMI3XY_SensorAPI/bmi323_examples/common/common.c \
	sensor/BMM350_SensorAPI/bmm350.c \
	sensor/BMM350_SensorAPI/bmm350_oor.c \
    sensor/BMM350_SensorAPI/examples/common/common.c \

# All source files
SRC := $(CPP_SRC) $(C_SRC)

# Object files
OBJ := $(addprefix $(BUILDDIR)/,$(SRC))
OBJ := $(OBJ:.cpp=.o)
OBJ := $(OBJ:.c=.o)

# Dependency files
DEP := $(OBJ:.o=.d)

all: $(TARGET)

# Link
$(TARGET): $(OBJ)
	mkdir -p $(OUTDIR)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

# Compile C++ files
$(BUILDDIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile C files
$(BUILDDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Include dependency files
-include $(DEP)

clean:
	rm -rf $(OUTDIR)

run: all
	./$(TARGET)

.PHONY: all clean run