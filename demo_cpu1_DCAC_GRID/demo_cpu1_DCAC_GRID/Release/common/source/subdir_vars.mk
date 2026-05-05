################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../common/source/F2837xD_CodeStartBranch.asm \
../common/source/F2837xD_DBGIER.asm \
../common/source/F2837xD_usDelay.asm 

C_SRCS += \
../common/source/F2837xD_Adc.c \
../common/source/F2837xD_CpuTimers.c \
../common/source/F2837xD_DefaultISR.c \
../common/source/F2837xD_Dma.c \
../common/source/F2837xD_ECap.c \
../common/source/F2837xD_EPwm.c \
../common/source/F2837xD_EQep.c \
../common/source/F2837xD_Emif.c \
../common/source/F2837xD_Gpio.c \
../common/source/F2837xD_I2C.c \
../common/source/F2837xD_Ipc.c \
../common/source/F2837xD_Ipc_Driver.c \
../common/source/F2837xD_Ipc_Driver_Lite.c \
../common/source/F2837xD_Ipc_Driver_Util.c \
../common/source/F2837xD_Mcbsp.c \
../common/source/F2837xD_PieCtrl.c \
../common/source/F2837xD_PieVect.c \
../common/source/F2837xD_Sci.c \
../common/source/F2837xD_Spi.c \
../common/source/F2837xD_SysCtrl.c \
../common/source/F2837xD_TempSensorConv.c \
../common/source/F2837xD_Upp.c \
../common/source/F2837xD_can.c \
../common/source/F2837xD_sci_io.c \
../common/source/F2837xD_sdfm_drivers.c \
../common/source/F2837xD_struct.c 

C_DEPS += \
./common/source/F2837xD_Adc.d \
./common/source/F2837xD_CpuTimers.d \
./common/source/F2837xD_DefaultISR.d \
./common/source/F2837xD_Dma.d \
./common/source/F2837xD_ECap.d \
./common/source/F2837xD_EPwm.d \
./common/source/F2837xD_EQep.d \
./common/source/F2837xD_Emif.d \
./common/source/F2837xD_Gpio.d \
./common/source/F2837xD_I2C.d \
./common/source/F2837xD_Ipc.d \
./common/source/F2837xD_Ipc_Driver.d \
./common/source/F2837xD_Ipc_Driver_Lite.d \
./common/source/F2837xD_Ipc_Driver_Util.d \
./common/source/F2837xD_Mcbsp.d \
./common/source/F2837xD_PieCtrl.d \
./common/source/F2837xD_PieVect.d \
./common/source/F2837xD_Sci.d \
./common/source/F2837xD_Spi.d \
./common/source/F2837xD_SysCtrl.d \
./common/source/F2837xD_TempSensorConv.d \
./common/source/F2837xD_Upp.d \
./common/source/F2837xD_can.d \
./common/source/F2837xD_sci_io.d \
./common/source/F2837xD_sdfm_drivers.d \
./common/source/F2837xD_struct.d 

OBJS += \
./common/source/F2837xD_Adc.obj \
./common/source/F2837xD_CodeStartBranch.obj \
./common/source/F2837xD_CpuTimers.obj \
./common/source/F2837xD_DBGIER.obj \
./common/source/F2837xD_DefaultISR.obj \
./common/source/F2837xD_Dma.obj \
./common/source/F2837xD_ECap.obj \
./common/source/F2837xD_EPwm.obj \
./common/source/F2837xD_EQep.obj \
./common/source/F2837xD_Emif.obj \
./common/source/F2837xD_Gpio.obj \
./common/source/F2837xD_I2C.obj \
./common/source/F2837xD_Ipc.obj \
./common/source/F2837xD_Ipc_Driver.obj \
./common/source/F2837xD_Ipc_Driver_Lite.obj \
./common/source/F2837xD_Ipc_Driver_Util.obj \
./common/source/F2837xD_Mcbsp.obj \
./common/source/F2837xD_PieCtrl.obj \
./common/source/F2837xD_PieVect.obj \
./common/source/F2837xD_Sci.obj \
./common/source/F2837xD_Spi.obj \
./common/source/F2837xD_SysCtrl.obj \
./common/source/F2837xD_TempSensorConv.obj \
./common/source/F2837xD_Upp.obj \
./common/source/F2837xD_can.obj \
./common/source/F2837xD_sci_io.obj \
./common/source/F2837xD_sdfm_drivers.obj \
./common/source/F2837xD_struct.obj \
./common/source/F2837xD_usDelay.obj 

ASM_DEPS += \
./common/source/F2837xD_CodeStartBranch.d \
./common/source/F2837xD_DBGIER.d \
./common/source/F2837xD_usDelay.d 

OBJS__QUOTED += \
"common\source\F2837xD_Adc.obj" \
"common\source\F2837xD_CodeStartBranch.obj" \
"common\source\F2837xD_CpuTimers.obj" \
"common\source\F2837xD_DBGIER.obj" \
"common\source\F2837xD_DefaultISR.obj" \
"common\source\F2837xD_Dma.obj" \
"common\source\F2837xD_ECap.obj" \
"common\source\F2837xD_EPwm.obj" \
"common\source\F2837xD_EQep.obj" \
"common\source\F2837xD_Emif.obj" \
"common\source\F2837xD_Gpio.obj" \
"common\source\F2837xD_I2C.obj" \
"common\source\F2837xD_Ipc.obj" \
"common\source\F2837xD_Ipc_Driver.obj" \
"common\source\F2837xD_Ipc_Driver_Lite.obj" \
"common\source\F2837xD_Ipc_Driver_Util.obj" \
"common\source\F2837xD_Mcbsp.obj" \
"common\source\F2837xD_PieCtrl.obj" \
"common\source\F2837xD_PieVect.obj" \
"common\source\F2837xD_Sci.obj" \
"common\source\F2837xD_Spi.obj" \
"common\source\F2837xD_SysCtrl.obj" \
"common\source\F2837xD_TempSensorConv.obj" \
"common\source\F2837xD_Upp.obj" \
"common\source\F2837xD_can.obj" \
"common\source\F2837xD_sci_io.obj" \
"common\source\F2837xD_sdfm_drivers.obj" \
"common\source\F2837xD_struct.obj" \
"common\source\F2837xD_usDelay.obj" 

C_DEPS__QUOTED += \
"common\source\F2837xD_Adc.d" \
"common\source\F2837xD_CpuTimers.d" \
"common\source\F2837xD_DefaultISR.d" \
"common\source\F2837xD_Dma.d" \
"common\source\F2837xD_ECap.d" \
"common\source\F2837xD_EPwm.d" \
"common\source\F2837xD_EQep.d" \
"common\source\F2837xD_Emif.d" \
"common\source\F2837xD_Gpio.d" \
"common\source\F2837xD_I2C.d" \
"common\source\F2837xD_Ipc.d" \
"common\source\F2837xD_Ipc_Driver.d" \
"common\source\F2837xD_Ipc_Driver_Lite.d" \
"common\source\F2837xD_Ipc_Driver_Util.d" \
"common\source\F2837xD_Mcbsp.d" \
"common\source\F2837xD_PieCtrl.d" \
"common\source\F2837xD_PieVect.d" \
"common\source\F2837xD_Sci.d" \
"common\source\F2837xD_Spi.d" \
"common\source\F2837xD_SysCtrl.d" \
"common\source\F2837xD_TempSensorConv.d" \
"common\source\F2837xD_Upp.d" \
"common\source\F2837xD_can.d" \
"common\source\F2837xD_sci_io.d" \
"common\source\F2837xD_sdfm_drivers.d" \
"common\source\F2837xD_struct.d" 

ASM_DEPS__QUOTED += \
"common\source\F2837xD_CodeStartBranch.d" \
"common\source\F2837xD_DBGIER.d" \
"common\source\F2837xD_usDelay.d" 

C_SRCS__QUOTED += \
"../common/source/F2837xD_Adc.c" \
"../common/source/F2837xD_CpuTimers.c" \
"../common/source/F2837xD_DefaultISR.c" \
"../common/source/F2837xD_Dma.c" \
"../common/source/F2837xD_ECap.c" \
"../common/source/F2837xD_EPwm.c" \
"../common/source/F2837xD_EQep.c" \
"../common/source/F2837xD_Emif.c" \
"../common/source/F2837xD_Gpio.c" \
"../common/source/F2837xD_I2C.c" \
"../common/source/F2837xD_Ipc.c" \
"../common/source/F2837xD_Ipc_Driver.c" \
"../common/source/F2837xD_Ipc_Driver_Lite.c" \
"../common/source/F2837xD_Ipc_Driver_Util.c" \
"../common/source/F2837xD_Mcbsp.c" \
"../common/source/F2837xD_PieCtrl.c" \
"../common/source/F2837xD_PieVect.c" \
"../common/source/F2837xD_Sci.c" \
"../common/source/F2837xD_Spi.c" \
"../common/source/F2837xD_SysCtrl.c" \
"../common/source/F2837xD_TempSensorConv.c" \
"../common/source/F2837xD_Upp.c" \
"../common/source/F2837xD_can.c" \
"../common/source/F2837xD_sci_io.c" \
"../common/source/F2837xD_sdfm_drivers.c" \
"../common/source/F2837xD_struct.c" 

ASM_SRCS__QUOTED += \
"../common/source/F2837xD_CodeStartBranch.asm" \
"../common/source/F2837xD_DBGIER.asm" \
"../common/source/F2837xD_usDelay.asm" 


