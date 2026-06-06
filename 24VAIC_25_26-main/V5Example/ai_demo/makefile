# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = src/ai_functions.cpp
SRC_C += src/ai_jetson.cpp
SRC_C += src/ai_robot_link.cpp
SRC_C += src/dashboard.cpp
SRC_C += src/main.cpp
SRC_C += src/skills_autonomous.cpp

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(notdir $(SRC_C)))) )

# location of include files that c and cpp files depend on
SRC_H  = include/ai_functions.h
SRC_H += include/ai_jetson.h
SRC_H += include/ai_robot_link.h
SRC_H += include/robot-config.h
SRC_H += include/skills_autonomous.h
SRC_H += include/vex.h

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# explicit compile rules for each source file (workaround for GNU Make 4.2 VPATH pattern rule bug on Windows)
$(BUILD)/ai_functions.o: src/ai_functions.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

$(BUILD)/ai_jetson.o: src/ai_jetson.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

$(BUILD)/ai_robot_link.o: src/ai_robot_link.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

$(BUILD)/dashboard.o: src/dashboard.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

$(BUILD)/main.o: src/main.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

$(BUILD)/skills_autonomous.o: src/skills_autonomous.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

# include build rules (link, bin, clean)
include vex/mkrules.mk
