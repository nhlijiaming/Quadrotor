################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --include_path="D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="D:/Program Files/TI CCS/Tiva C Firmware/driverlib" --include_path="D:/Program Files/TI CCS/Tiva C Firmware" -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

tm4c1233h6pm_startup_ccs.obj: ../tm4c1233h6pm_startup_ccs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --include_path="D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="D:/Program Files/TI CCS/Tiva C Firmware/driverlib" --include_path="D:/Program Files/TI CCS/Tiva C Firmware" -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="tm4c1233h6pm_startup_ccs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


