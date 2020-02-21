################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32MG12P/GCC/subdir.mk
-include platform/service/sleeptimer/src/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32MG12P/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include app/bluetooth/common/util/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: soc-bluvision-beek.axf

# Tool invocations
soc-bluvision-beek.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/efr32mg12p332f1024gl125.ld" -Xlinker --gc-sections -Xlinker -Map="soc-bluvision-beek.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o soc-bluvision-beek.axf -Wl,--start-group "./app/bluetooth/common/util/infrastructure.o" "./application_properties.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./led_control.o" "./led_driver.o" "./main.o" "./pti.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/si7013.o" "./hardware/kit/common/drivers/tempsens.o" "./hardware/kit/common/drivers/udelay.o" "./platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/startup_efr32mg12p.o" "./platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_burtc.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_eusart.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./platform/service/sleeptimer/src/sl_sleeptimer.o" "./platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/lib/EFR32MG12P/GCC/binapploader.o" "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/platform/radio/rail_lib/autogen/librail_release/librail_efr32xg12_gcc_release.a" "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/lib/EFR32MG12P/GCC/libbluetooth.a" "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/lib/EFR32MG12P/GCC/libpsstore.a" "/home/blitz/SimplicityStudio/v4_workspace/soc-bluvision-beek/protocol/bluetooth/lib/EFR32MG12P/GCC/libmbedtls.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc-bluvision-beek.hex'
	arm-none-eabi-objcopy -O ihex "soc-bluvision-beek.axf" "soc-bluvision-beek.hex"
	@echo ' '

	@echo 'Building bin file: soc-bluvision-beek.bin'
	arm-none-eabi-objcopy -O binary "soc-bluvision-beek.axf" "soc-bluvision-beek.bin"
	@echo ' '

	@echo 'Building s37 file: soc-bluvision-beek.s37'
	arm-none-eabi-objcopy -O srec "soc-bluvision-beek.axf" "soc-bluvision-beek.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc-bluvision-beek.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) soc-bluvision-beek.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
