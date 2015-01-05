# Put your STM32F4 library code directory here
#STM_COMMON=./STM32F10x_StdPeriph_Driver
STM_COMMON=./stsw-m24sr001_v1.3.0

# Put your stlink folder here so make burn will work.
STLINK=/home/dahl/bin/stlink

# Put your source files here (or *.c, etc)
# drv_lcdspi_ili9341.c
SRCS= hw_config.c main.c stm32f10x_it.c drv_lcdspi_ili9341.c

SRCS += $(wildcard $(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/src/*.c)
SRCS += $(wildcard $(STM_COMMON)/Utilities/M24SR_DISCOVERY_MB1138/LED/src/*.c)
# I'm using a tweaked version of the LCD driver.
#SRCS += $(wildcard $(STM_COMMON)/Utilities/M24SR_DISCOVERY_MB1138/LCD/src/*.c)


#SRCS += STM32F10x_StdPeriph_Driver/src/misc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c

SRCS += ./stsw-m24sr001_v1.3.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c

#main.c system_stm32f4xx.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=break

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall

# Linker script
CFLAGS += -Tstm32_flash.ld
# doesn't work
#CFLAGS += -Tstm32f103.ld
# doesn't work
#CFLAGS += -Ttruestudio_stm32_flash_template.ld

CFLAGS += -std=c99
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -lm
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F10x_StdPeriph_Driver/inc
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
CFLAGS += -I$(STM_COMMON)/Utilities/M24SR_DISCOVERY_MB1138/LED/inc/

#CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
#CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
#CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc

# add startup file to build
#SRCS += $(STM_COMMON)/Libraries/CMSIS/Core/CM3/startup/arm/startup_stm32f10x_hd.s
#SRCS += $(STM_COMMON)/Libraries/CMSIS/Core/CM3/startup/gcc/startup_stm32f10x_hd.c
#SRCS += $(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s

#SRCS += startup_stm32f10x_xl.s
# most promising. probably the right one
SRCS += startup_stm32f10x_xl.s

#SRCS += old_startup_STM32F10x.s # incorrect format 

#SRCS += startup_stm32f10x_hd.s
OBJS = $(SRCS:.c=.o)


.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash
burn: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x08000000
