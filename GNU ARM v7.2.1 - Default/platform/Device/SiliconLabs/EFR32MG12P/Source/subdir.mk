################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.c 

OBJS += \
./platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o 

C_DEPS += \
./platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.d 


# Each subdirectory must supply rules for building sources it contributes
platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o: ../platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.d" -MT"platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


