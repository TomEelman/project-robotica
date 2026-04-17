CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -I.

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
	$(CXX) $(CXXFLAGS) -o $@ $^
	@rm -f $(OBJS)
	@echo "Build klaar, .o bestanden verwijderd."

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@rm -f *.o $(TARGET)
	@echo "Opgeruimd: .o bestanden en $(TARGET) verwijderd."

.PHONY: all clean