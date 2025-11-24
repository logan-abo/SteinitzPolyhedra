# ==========================================
# Project Settings
# ==========================================

TARGET_EXEC := steinitz_solver

# Compiler
CXX := g++

# 1. EIGEN CONFIGURATION (Option B)
# Pointing to C:/eigen-5.0.0 via the WSL mount point
# Verify capitalization: Linux is case-sensitive!
EIGEN_PATH := /mnt/c/eigen-5.0.0

# Compiler Flags
# -I$(EIGEN_PATH) adds your Windows folder to the include path
CXXFLAGS := -std=c++17 -Wall -Wextra -g -MMD -MP -I$(EIGEN_PATH)

# Linker Flags
# Since you installed SFML via apt, Linux knows where the libs are.
LDFLAGS  := -lsfml-graphics -lsfml-window -lsfml-system

# ==========================================
# Directory Layout
# ==========================================

SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

# ==========================================
# Automated Discovery
# ==========================================

# Find all .cpp files
SRCS := $(shell find $(SRC_DIR) -name "*.cpp")

# Create Object list
OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Auto-discover "Interfaces" folders for headers
INC_DIRS := $(shell find $(SRC_DIR) -type d -name "Interfaces")
INC_DIRS += $(SRC_DIR)

# Add -I prefix
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

# ==========================================
# Build Rules
# ==========================================

all: $(BIN_DIR)/$(TARGET_EXEC)

$(BIN_DIR)/$(TARGET_EXEC): $(OBJS)
	@mkdir -p $(dir $@)
	@echo "Linking $@"
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	$(CXX) $(CXXFLAGS) $(INC_FLAGS) -c $< -o $@

clean:
	@echo "Cleaning..."
	rm -rf $(OBJ_DIR) $(BIN_DIR)

-include $(OBJS:.o=.d)

.PHONY: all clean