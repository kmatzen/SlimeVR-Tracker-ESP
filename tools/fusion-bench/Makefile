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
#   make format     apply the repo's clang-format style
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

# Headers are prerequisites too. Without these, editing a header leaves a stale
# binary in place and `make test` reports a pass for code it never compiled --
# which it did, until a deliberate mutation failed to fail.
HEADERS   := $(wildcard src/*.h) $(VQF_DIR)/vqf.h \
             $(wildcard $(ROOT)/src/sensors/softfusion/drivers/magfifo.h) \
             $(wildcard $(ROOT)/src/sensors/softfusion/drivers/bmm350comp.h)

SOURCES := src/main.cpp src/dataset.cpp src/metrics.cpp src/synth.cpp \
           src/dataset.h src/metrics.h src/quatmath.h src/synth.h \
           tests/selftest.cpp

# The repo's `format` CI job runs clang-format 17 over the whole tree, and
# formatting differs between clang-format versions -- so pin it. `pip install
# clang-format==17.0.6` gives the exact binary CI uses.
CLANG_FORMAT ?= clang-format

.PHONY: all test suite check baseline format format-check clean

all: $(BUILD)/fusion-bench $(BUILD)/selftest

$(BUILD)/fusion-bench: $(BENCH_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $(BENCH_SRC)

$(BUILD)/selftest: $(TEST_SRC) $(HEADERS) | $(BUILD)
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

format:
	$(CLANG_FORMAT) -i $(SOURCES)

format-check:
	$(CLANG_FORMAT) --dry-run --Werror $(SOURCES)

clean:
	rm -rf $(BUILD)
