################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
User/source/%.obj: ../User/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"E:/Program_2024/CCS12.8.1/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 -Ooff --opt_for_speed=2 --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/lib/basic/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/Add/json" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/App/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/User/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/Dsp/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/Add" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/lib/rts2800_fpu32/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/common/include" --include_path="C:/Users/admin/Desktop/文件夹/demo_cpu1_DCAC_GRID/demo_cpu1_DCAC_GRID/headers/include" --include_path="E:/Program_2024/CCS12.8.1/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=CPU1 --define=_FLASH --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="User/source/$(basename $(<F)).d_raw" --obj_directory="User/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


