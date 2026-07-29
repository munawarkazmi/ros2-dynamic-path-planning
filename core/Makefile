# Builds the ROS-free planning core: tests and (later) the benchmark.
# Requires only g++ with C++20 support.
CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  = -Iinclude

BUILD := build

SRCS  := src/astar.cpp src/dstar_lite.cpp
OBJS  := $(SRCS:src/%.cpp=$(BUILD)/%.o)

.PHONY: all test clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/test_planners: tests/test_planners.cpp $(OBJS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(OBJS) -o $@

test: $(BUILD)/test_planners
	./$(BUILD)/test_planners

clean:
	rm -rf $(BUILD)
