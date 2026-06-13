CXX = g++

CXXFLAGS = -Wall -std=c++20

TARGET=MUNIN_POINTING

all: $(TARGET)

$(TARGET): pointing.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) pointing.o

pointing.o: pointing.cpp
	$(CXX) $(CXXFLAGS) -c pointing.cpp

clean: 
	rm -f *.o $(TARGET)