################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/CMSIS-SP-00300-r4p5-00rel0/CMSIS/Include" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/driverlib" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/inc" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/utils" -g --gcc --define=ccs="ccs" --define=__FPU_PRESENT=1 --define=ARM_MATH_CM4 --define=PART_TM4C129ENCPDT --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

tm4c129encpdt_startup_ccs.obj: ../tm4c129encpdt_startup_ccs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.6/include" --include_path="C:/CMSIS-SP-00300-r4p5-00rel0/CMSIS/Include" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/driverlib" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/inc" --include_path="C:/ti/TivaWare_C_Series-2.1.1.71/utils" -g --gcc --define=ccs="ccs" --define=__FPU_PRESENT=1 --define=ARM_MATH_CM4 --define=PART_TM4C129ENCPDT --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="tm4c129encpdt_startup_ccs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


