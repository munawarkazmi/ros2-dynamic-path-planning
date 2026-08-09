# Builds the ROS-free planning core: tests and (later) the benchmark.
# Requires only g++ with C++20 support.
CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  = -Iinclude

BUILD := build

SRCS  := src/astar.cpp src/dstar_lite.cpp
OBJS  := $(SRCS:src/%.cpp=$(BUILD)/%.o)

.PHONY: all test fuzz benchmark clean

all: test fuzz

$(BUILD)/fuzz_planners: tests/fuzz_planners.cpp $(OBJS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(OBJS) -o $@

# The counts this prints are quoted by the README and by the site, and until
# now they lived only in whatever terminal last ran it. Nothing could check
# them against anything, which made them the one figure on the site with no
# record behind it. Recorded here instead.
#
# Written through a temporary file so a failed run cannot leave behind a
# summary describing a clean one. The binary exits non-zero when a replan
# disagrees with Dijkstra, make stops at that line, and the committed summary
# goes on describing the last run that actually passed.
FUZZ_REPORT := ../reports/results/fuzz_summary.txt

fuzz: $(BUILD)/fuzz_planners
	./$(BUILD)/fuzz_planners > $(FUZZ_REPORT).tmp
	@mv -f $(FUZZ_REPORT).tmp $(FUZZ_REPORT)
	@cat $(FUZZ_REPORT)

benchmark: $(BUILD)/benchmark
	@echo "run: $(BUILD)/benchmark --map ../maps/indoor_grid.pgm --out ../reports/results/replan_benchmark.csv"

$(BUILD)/benchmark: benchmark/benchmark_main.cpp $(OBJS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(OBJS) -o $@

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
