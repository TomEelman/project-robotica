CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wconversion -I.

RPLIDAR_SDK = $(HOME)/rplidar_sdk
RPLIDAR_INC = -I$(RPLIDAR_SDK)/sdk/include -I$(RPLIDAR_SDK)/sdk/src
RPLIDAR_LIB = -L$(RPLIDAR_SDK)/output/Linux/Release -lsl_lidar_sdk

TARGET = robocar_pi5

SRCS =  MainPi5.cpp       \
        LIDAR.cpp         \
        Mapper.cpp        \
        GridMap.cpp       \
        Position.cpp      \
        ExplorationPlanner.cpp \
        PathPlanner.cpp   \
        Navigator.cpp     \
        Path.cpp

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(RPLIDAR_LIB) -lstdc++ -lpthread
	@rm -f $(OBJS)
	@echo "Build klaar, .o bestanden verwijderd."

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(RPLIDAR_INC) -c $< -o $@

clean:
	@rm -f *.o $(TARGET)
	@echo "Opgeruimd: .o bestanden en $(TARGET) verwijderd."

.PHONY: all clean