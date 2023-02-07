CXX      := -c++
CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror -Wno-deprecated-copy -std=c++17 -march=native -g -O3 -flto
LDFLAGS  := -L/usr/lib -lstdc++ -lm
BUILD    := ./build
OBJ_DIR  := $(BUILD)/objects
TARGET   := run_filter
INCLUDE  := -Ieigen
SRC      := replay.cpp BatteryEKF.cpp

OBJECTS := $(SRC:%.cpp=$(OBJ_DIR)/%.o)

all: build $(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o "$@" -c "$<"

$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LDFLAGS) -o $(TARGET) $(OBJECTS)

.PHONY: all build clean debug release

build:
	@mkdir -p $(OBJ_DIR)

clean:
	-@rm -rvf $(BUILD)
	-@rm -rvf $(TARGET)
	-@rm -rvf ekf_generated

depend: .depend

.depend: $(SRC)
	rm -f "$@"
	$(CXX) $(CXXFLAGS) -MM $^ -MF "$@"

include .depend

$(OBJ_DIR)/BatteryEKF.o: ekf_generated/init.cpp ekf_generated/predict.cpp ekf_generated/update.cpp

ekf_generated/%.cpp: derivations/%.py derivations/common.py derivations/helpers.py
	@mkdir -p "$(@D)"
	python "$<" "$@"
