#!/bin/bash
# TREK1000 Adapted Project - all builds from station source
set -e

SCRIPT_DIR="C:/Users/cheng/Desktop/homework/无人机/main/trek1000_adapted"
DOWNLOAD_DIR="C:/Users/cheng/Desktop/homework/无人机/Download"
MIG_DIR="C:/Users/cheng/Desktop/homework/无人机/main/trek1000_migration/port_adapted"
SRC="$DOWNLOAD_DIR/UWB_TDOA_STATION_V1_1-master/UWB_TDOA_STATION_V1_1-master/Code/gataway20181020/test_rx11"

echo "=== TREK1000 Adapted Project Setup ==="

# ── 1. clean ──
echo "[1/3] Cleaning..."
rm -rf "$SCRIPT_DIR/tag" "$SCRIPT_DIR/anchor_a0" "$SCRIPT_DIR/anchor_a1" "$SCRIPT_DIR/anchor_a2" "$SCRIPT_DIR/anchor_a3"

# ── 2. copy from station source ──
echo "[2/3] Copying from station source..."
for target in tag anchor_a0 anchor_a1 anchor_a2 anchor_a3; do
    cp -r "$SRC" "$SCRIPT_DIR/$target"
    rm -rf "$SCRIPT_DIR/$target/OBJ" 2>/dev/null || true
    echo "  $target copied"
done

# ── 3. apply adaptations ──
echo "[3/3] Applying adaptations..."

for target in tag anchor_a0 anchor_a1 anchor_a2 anchor_a3; do
    T="$SCRIPT_DIR/$target"
    # port.h + stm32f10x_it.c + spi.h + led.c
    cp "$MIG_DIR/port.h"         "$T/HARDWARE/PORT/port.h"
    cp "$MIG_DIR/stm32f10x_it.c" "$T/HARDWARE/platform/stm32f10x_it.c"
    cp "$MIG_DIR/spi.h"          "$T/HARDWARE/SPI/spi.h"
    cp "$MIG_DIR/led.c"          "$T/HARDWARE/LED/led.c"
    # 16MHz HSE
    sed -i 's/RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9)/RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9)/' "$T/HARDWARE/PORT/port.c"
done

# role-specific s1switch
sed -i 's/s1switch = 0x[0-9a-fA-F]*/s1switch = 0x04;  \/* TAG mode *\//' "$SCRIPT_DIR/tag/HARDWARE/application/main.c"
sed -i 's/s1switch = 0x[0-9a-fA-F]*/s1switch = 0x0C;  \/* Anchor A0 gateway *\//' "$SCRIPT_DIR/anchor_a0/HARDWARE/application/main.c"
sed -i 's/s1switch = 0x[0-9a-fA-F]*/s1switch = 0x1C;  \/* Anchor A1 *\//' "$SCRIPT_DIR/anchor_a1/HARDWARE/application/main.c"
sed -i 's/s1switch = 0x[0-9a-fA-F]*/s1switch = 0x2C;  \/* Anchor A2 *\//' "$SCRIPT_DIR/anchor_a2/HARDWARE/application/main.c"
sed -i 's/s1switch = 0x[0-9a-fA-F]*/s1switch = 0x6C;  \/* Anchor A3 *\//' "$SCRIPT_DIR/anchor_a3/HARDWARE/application/main.c"

# verify
echo ""
echo "=== Verification ==="
for target in tag anchor_a0 anchor_a1 anchor_a2 anchor_a3; do
    SW=$(grep 's1switch = 0x' "$SCRIPT_DIR/$target/HARDWARE/application/main.c" | head -1 | grep -oP '0x[0-9a-fA-F]+')
    RSTN=$(grep -c 'DW1000_RSTn.*GPIO_Pin_4' "$SCRIPT_DIR/$target/HARDWARE/PORT/port.h")
    IRQ=$(grep -c 'DECAIRQ.*GPIO_Pin_0' "$SCRIPT_DIR/$target/HARDWARE/PORT/port.h")
    HSE=$(grep -c 'HSE_Div2' "$SCRIPT_DIR/$target/HARDWARE/PORT/port.c")
    echo "  $target: s1switch=$SW  RSTn=$RSTN  IRQ=$IRQ  HSE=$HSE"
done

echo ""
echo "=== Setup Complete ==="
echo "  trek1000_adapted/tag/         TAG (drone, s1switch=0x04)"
echo "  trek1000_adapted/anchor_a0/   Anchor A0 gateway (s1switch=0x0C)"
echo "  trek1000_adapted/anchor_a1/   Anchor A1 (s1switch=0x1C)"
echo "  trek1000_adapted/anchor_a2/   Anchor A2 (s1switch=0x2C)"
echo "  trek1000_adapted/anchor_a3/   Anchor A3 (s1switch=0x6C)"
echo ""
echo "All 5 builds from SAME station source. Only s1switch differs."
