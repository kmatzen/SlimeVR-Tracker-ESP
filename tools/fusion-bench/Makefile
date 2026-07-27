# Host build for the fusion benchmark.
#
# Deliberately a plain Makefile with no dependencies beyond a C++17 compiler:
# the point of this tool is that anyone can run it, including CI, without
# installing PlatformIO or an embedded toolchain.
#
#   make            build fusion-bench and selftest
#   make test       build and run the self-tests
#   make suite      run the standard benchmark suite
#   make check      run the suite against the committed baseline (CI gate)
#   make baseline   regenerate the baseline file
#   make clean

ROOT     := ../..
VQF_DIR  := $(ROOT)/lib/vqf

CXX      ?= c++
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -Isrc -I$(VQF_DIR)

BUILD    := build

BENCH_SRC := src/main.cpp src/dataset.cpp src/metrics.cpp src/synth.cpp \
             $(VQF_DIR)/vqf.cpp
TEST_SRC  := tests/selftest.cpp src/dataset.cpp src/metrics.cpp src/synth.cpp \
             $(VQF_DIR)/vqf.cpp

.PHONY: all test suite check baseline clean

all: $(BUILD)/fusion-bench $(BUILD)/selftest

$(BUILD)/fusion-bench: $(BENCH_SRC) | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $(BENCH_SRC)

$(BUILD)/selftest: $(TEST_SRC) | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $(TEST_SRC)

$(BUILD):
	mkdir -p $(BUILD)

test: $(BUILD)/selftest
	./$(BUILD)/selftest

suite: $(BUILD)/fusion-bench
	./$(BUILD)/fusion-bench suite

check: $(BUILD)/selftest $(BUILD)/fusion-bench
	./$(BUILD)/selftest
	./$(BUILD)/fusion-bench suite --baseline baseline.txt --json $(BUILD)/report.json

baseline: $(BUILD)/fusion-bench
	./$(BUILD)/fusion-bench suite --write-baseline baseline.txt

clean:
	rm -rf $(BUILD)
