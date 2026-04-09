#!/bin/bash
set -e

echo "== Building STM32F103 firmware =="

# Toolchain

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

# Flags

CFLAGS="-mcpu=cortex-m3 -mthumb -O0 -g -ffunction-sections -fdata-sections -Wall"
DEFINES="-DSTM32F103xB"
INCLUDES="-Iinc -Icmsis -Isrc"

# Linker

LDSCRIPT=stm32f103rb.ld

# Output

OUT=build/firmware

mkdir -p build

echo "== Compiling =="

$CC $CFLAGS $DEFINES $INCLUDES -c src/main.c -o build/main.o
$CC $CFLAGS $DEFINES $INCLUDES -c src/system_stm32f1xx.c -o build/system.o
$CC -mcpu=cortex-m3 -mthumb -c src/startup_stm32f103xb.s -o build/startup.o

echo "== Linking =="

$CC -mcpu=cortex-m3 -mthumb build/main.o build/system.o build/startup.o -T $LDSCRIPT -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-Map=build/firmware.map -o $OUT.elf

echo "== Size =="

$SIZE $OUT.elf

echo "== Generating binary/hex =="

$OBJCOPY -O binary $OUT.elf $OUT.bin
$OBJCOPY -O ihex   $OUT.elf $OUT.hex

echo "== Flashing =="

st-flash --connect-under-reset write $OUT.bin 0x08000000

echo "== Done =="
