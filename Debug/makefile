################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./userLib/board.obj" \
"./userLib/Voltmeter.obj" \
"./userLib/Sonar.obj" \
"./userLib/PPM_Receiver.obj" \
"./userLib/PPM_Encoder.obj" \
"./userLib/PID.obj" \
"./userLib/OpticalFlow.obj" \
"./userLib/OLED.obj" \
"./userLib/MadgwickAHRS.obj" \
"./userLib/MPU9250.obj" \
"./userLib/Interface.obj" \
"./userLib/IMUFilter.obj" \
"./userLib/EEPROM.obj" \
"./userLib/Debugger.obj" \
"./userLib/ClockCounter.obj" \
"./utils/ustdlib.obj" \
"./utils/uartstdio.obj" \
"./utils/cmdline.obj" \
"./tm4c1233h6pm_startup_ccs.obj" \
"./main.obj" \
"../tm4c1233h6pm.cmd" \
-l"libc.a" \
-l"D:/Program Files/TI CCS/Tiva C Firmware/examples/project/ccs/../../../driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include userLib/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
-include userLib/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Quadcopter.out \

EXE_OUTPUTS__QUOTED += \
"Quadcopter.out" \

BIN_OUTPUTS += \
Quadcopter.hex \

BIN_OUTPUTS__QUOTED += \
"Quadcopter.hex" \


# All Target
all: Quadcopter.out

# Tool invocations
Quadcopter.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual -z -m"project_ccs.map" --heap_size=0x1500 --stack_size=0x1500 -i"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/lib" -i"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="Quadcopter_linkInfo.xml" --rom_model -o "Quadcopter.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Quadcopter.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armhex"  -o "Quadcopter.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "main.pp" "tm4c1233h6pm_startup_ccs.pp" "utils\cmdline.pp" "utils\uartstdio.pp" "utils\ustdlib.pp" "userLib\ClockCounter.pp" "userLib\Debugger.pp" "userLib\EEPROM.pp" "userLib\IMUFilter.pp" "userLib\Interface.pp" "userLib\MPU9250.pp" "userLib\MadgwickAHRS.pp" "userLib\OLED.pp" "userLib\OpticalFlow.pp" "userLib\PID.pp" "userLib\PPM_Encoder.pp" "userLib\PPM_Receiver.pp" "userLib\Sonar.pp" "userLib\Voltmeter.pp" "userLib\board.pp" 
	-$(RM) "main.obj" "tm4c1233h6pm_startup_ccs.obj" "utils\cmdline.obj" "utils\uartstdio.obj" "utils\ustdlib.obj" "userLib\ClockCounter.obj" "userLib\Debugger.obj" "userLib\EEPROM.obj" "userLib\IMUFilter.obj" "userLib\Interface.obj" "userLib\MPU9250.obj" "userLib\MadgwickAHRS.obj" "userLib\OLED.obj" "userLib\OpticalFlow.obj" "userLib\PID.obj" "userLib\PPM_Encoder.obj" "userLib\PPM_Receiver.obj" "userLib\Sonar.obj" "userLib\Voltmeter.obj" "userLib\board.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

