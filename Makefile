CXX = g++
CXXFLAGS = -Wall -g -Iinclude -MMD -MP
LDFLAGS = -lgpiodcxx

OUTDIR = outputs
BUILDDIR = $(OUTDIR)/build

TARGET = $(OUTDIR)/main

# Source files
SRC = $(wildcard src/*.cpp)

# Object files
OBJ = $(SRC:src/%.cpp=$(BUILDDIR)/%.o)

# Dependency files
DEP = $(OBJ:.o=.d)

all: $(TARGET)

# Link
$(TARGET): $(OBJ)
	mkdir -p $(OUTDIR)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

# Compile
$(BUILDDIR)/%.o: src/%.cpp
	mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Include auto-generated dependencies
-include $(DEP)

clean:
	rm -rf $(OUTDIR)

run: all
	./$(TARGET)

.PHONY: all clean run