################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
utils/cmdline.obj: D:/Program\ Files/TI\ CCS/Tiva\ C\ Firmware/utils/cmdline.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --include_path="D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="D:/Program Files/TI CCS/Tiva C Firmware/driverlib" --include_path="D:/Program Files/TI CCS/Tiva C Firmware" -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="utils/cmdline.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/uartstdio.obj: D:/Program\ Files/TI\ CCS/Tiva\ C\ Firmware/utils/uartstdio.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --include_path="D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="D:/Program Files/TI CCS/Tiva C Firmware/driverlib" --include_path="D:/Program Files/TI CCS/Tiva C Firmware" -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="utils/uartstdio.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/ustdlib.obj: D:/Program\ Files/TI\ CCS/Tiva\ C\ Firmware/utils/ustdlib.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff --include_path="D:/Program Files/TI CCS/ccsv6/tools/compiler/arm_5.1.5/include" --include_path="D:/Program Files/TI CCS/Tiva C Firmware/driverlib" --include_path="D:/Program Files/TI CCS/Tiva C Firmware" -g --gcc --define=ccs="ccs" --define=PART_TM4C1233H6PM --define=TARGET_IS_TM4C123_RA3 --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="utils/ustdlib.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


