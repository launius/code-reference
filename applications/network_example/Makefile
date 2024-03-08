
CFLAGS = -g -Wall -O2
CXXFLAGS = -Wall -O2 -I./include -lwpa_client

TARGET = net_control

all: $(TARGET)

$(TARGET): $(TARGET).cpp 
	${CXX} $(CXXFLAGS) -L. -o $(TARGET) $(TARGET).cpp
