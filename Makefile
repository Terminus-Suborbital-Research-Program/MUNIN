CXX = g++
CXXFLAGS = -Wall -g -Iinclude
LDFLAGS = -lgpiodcxx

TARGET = main

# All source files
SRC = $(wildcard src/*.cpp)

# Object files (build/ keeps things clean)
OBJ = $(SRC:src/%.cpp=build/%.o)

all: $(TARGET)

# Link step
$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compile step
build/%.o: src/%.cpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build $(TARGET)

run: all
	./$(TARGET)