
#!/bin/bash
set -e

echo "============================== Building STM32F103 firmware ================================"

# Toolchain

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

# Flags

CFLAGS="
-mcpu=cortex-m3
-mthumb
-O0
-g
-ffunction-sections
-fdata-sections
-Wall
-Wextra
"

DEFINES="
-DSTM32F103xB
"

INCLUDES="
-Iinc
-Icmsis
-Isrc
-Irtos/include
"

# Linker

LDSCRIPT=stm32f103rb.ld

# Output

OUT=build/firmware

mkdir -p build

echo "======================================= Compiling ========================================="

# Automatically discover application and FreeRTOS sources
OBJECTS=()

for SRC in src/*.c rtos/*.c; do
    OBJ="build/${SRC%.c}.o"

    mkdir -p "$(dirname "$OBJ")"

    echo "Compiling $SRC"

    $CC $CFLAGS $DEFINES $INCLUDES \
        -c "$SRC" \
        -o "$OBJ"

    OBJECTS+=("$OBJ")
done

# Startup
$CC \
-mcpu=cortex-m3 -mthumb \
-c src/startup_stm32f103xb.s \
-o build/startup.o

OBJECTS+=("build/startup.o")

echo "======================================== Linking =========================================="

$CC \
-mcpu=cortex-m3 -mthumb \
"${OBJECTS[@]}" \
-T "$LDSCRIPT" \
-nostartfiles \
--specs=nosys.specs \
-Wl,--gc-sections \
-Wl,-Map=build/firmware.map \
-o "$OUT.elf"

echo "========================================= Size ============================================"

$SIZE "$OUT.elf"

echo "================================= Generating binary/hex ==================================="

$OBJCOPY -O binary "$OUT.elf" "$OUT.bin"
$OBJCOPY -O ihex   "$OUT.elf" "$OUT.hex"

echo "======================================= Flashing =========================================="

st-flash --connect-under-reset write "$OUT.bin" 0x08000000

echo "========================================= Done ============================================"
