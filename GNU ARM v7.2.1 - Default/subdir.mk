################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../application_properties.c \
../gatt_db.c \
../init_app.c \
../init_board.c \
../init_mcu.c \
../led_control.c \
../led_driver.c \
../main.c \
../pti.c 

OBJS += \
./application_properties.o \
./gatt_db.o \
./init_app.o \
./init_board.o \
./init_mcu.o \
./led_control.o \
./led_driver.o \
./main.o \
./pti.o 

C_DEPS += \
./application_properties.d \
./gatt_db.d \
./init_app.d \
./init_board.d \
./init_mcu.d \
./led_control.d \
./led_driver.d \
./main.d \
./pti.d 


# Each subdirectory must supply rules for building sources it contributes
application_properties.o: ../application_properties.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"application_properties.d" -MT"application_properties.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gatt_db.o: ../gatt_db.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"gatt_db.d" -MT"gatt_db.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

init_app.o: ../init_app.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"init_app.d" -MT"init_app.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

init_board.o: ../init_board.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"init_board.d" -MT"init_board.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

init_mcu.o: ../init_mcu.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"init_mcu.d" -MT"init_mcu.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

led_control.o: ../led_control.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"led_control.d" -MT"led_control.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

led_driver.o: ../led_driver.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"led_driver.d" -MT"led_driver.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.o: ../main.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"main.d" -MT"main.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

pti.o: ../pti.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__HEAP_SIZE=0xD00' '-D__STACK_SIZE=0x800' '-D__StackLimit=0x20000000' '-DEFR32MG12P332F1024GL125=1' -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/EFR32MG12_BRD4162A/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/CMSIS/Include" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emlib/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/halconfig" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/app/bluetooth/common/util" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/halconfig/inc/hal-config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/common" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/drivers" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/hardware/kit/common/bsp" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader/api" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/gpiointerrupt/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/src" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/sleep/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ble" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/common/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/ble_stack/inc/soc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/service/sleeptimer/config" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/emdrv/uartdrv/inc" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/Device/SiliconLabs/EFR32MG12P/Source" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/protocol/ieee802154" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek" -I"/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/bootloader" -O2 -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"pti.d" -MT"pti.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


