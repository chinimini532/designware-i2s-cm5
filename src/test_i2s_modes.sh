#!/bin/bash
# NGT I2S Configuration Test Script
# Tests different I2S modes to find the D5D5 pattern from FPGA
#
# Usage: ./test_i2s_modes.sh
#
# Make sure to run: sudo ./bin/fpga -w 0x0A 0x00 before testing

echo "=============================================="
echo "NGT I2S Configuration Test Script"
echo "=============================================="
echo ""
echo "This script will test different I2S configurations"
echo "to find the D5D5 pattern from FPGA."
echo ""
echo "Make sure FPGA is configured:"
echo "  sudo ./bin/fpga -w 0x0A 0x00"
echo ""

# Function to test a configuration
test_config() {
    local test_name="$1"
    local params="$2"

    echo "----------------------------------------------"
    echo "TEST: $test_name"
    echo "PARAMS: $params"
    echo "----------------------------------------------"

    sudo rmmod designware-i2s-NGT 2>/dev/null
    sleep 0.2
    sudo dmesg -C

    sudo modprobe designware_i2s_NGT $params

    sleep 0.5

    # Check for D5 pattern
    d5_count=$(dmesg | grep -c "0xd5\|0xD5\|D5")

    if [ "$d5_count" -gt 0 ]; then
        echo "*** FOUND D5 PATTERN! ($d5_count occurrences) ***"
        dmesg | grep -E "D5|0xd5|0xD5" | head -5
    else
        echo "No D5 pattern found"
    fi

    # Show first few samples
    echo ""
    echo "First 5 samples:"
    dmesg | grep "NGT_RX\[" | head -5

    echo ""
}

# Make sure module is unloaded
sudo rmmod designware-i2s-NGT 2>/dev/null

echo "=============================================="
echo "TEST GROUP 1: Different Resolutions"
echo "=============================================="

# Test 1: 16-bit resolution (0x01)
test_config "16-bit resolution (0x01), I2S standard" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_frame_offset=1 ngt_alaw_shift=8"

# Test 2: 20-bit resolution (0x02) - current default
test_config "20-bit resolution (0x02), I2S standard" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x02 ngt_frame_offset=1 ngt_alaw_shift=8"

# Test 3: 24-bit resolution (0x03)
test_config "24-bit resolution (0x03), I2S standard" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x03 ngt_frame_offset=1 ngt_alaw_shift=8"

# Test 4: 32-bit resolution (0x04)
test_config "32-bit resolution (0x04), I2S standard" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x04 ngt_frame_offset=1 ngt_alaw_shift=24"

echo "=============================================="
echo "TEST GROUP 2: Different Frame Offsets"
echo "=============================================="

# Test 5: Left-justified (frame_offset=0)
test_config "16-bit, Left-Justified (offset=0)" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_frame_offset=0 ngt_alaw_shift=8"

# Test 6: Left-justified with 32-bit
test_config "32-bit, Left-Justified (offset=0)" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x04 ngt_frame_offset=0 ngt_alaw_shift=24"

echo "=============================================="
echo "TEST GROUP 3: Different A-law Byte Positions"
echo "=============================================="

# Test 7-10: Try all byte positions with 32-bit mode
for shift in 0 8 16 24; do
    test_config "32-bit, A-law shift=$shift" \
        "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x04 ngt_frame_offset=1 ngt_alaw_shift=$shift"
done

echo "=============================================="
echo "TEST GROUP 4: TDM Mode"
echo "=============================================="

# Test 11: TDM mode with 2 slots
test_config "TDM mode, 2 slots, 16-bit" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_tdm_enable=1 ngt_tdm_slots=2 ngt_alaw_shift=8"

# Test 12: TDM mode with 4 slots
test_config "TDM mode, 4 slots, 16-bit" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_tdm_enable=1 ngt_tdm_slots=4 ngt_alaw_shift=8"

# Test 13: TDM mode with 8 slots
test_config "TDM mode, 8 slots, 32-bit" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x04 ngt_tdm_enable=1 ngt_tdm_slots=8 ngt_alaw_shift=8"

echo "=============================================="
echo "TEST GROUP 5: Channel Swap Variations"
echo "=============================================="

# Test 14: With channel swap
test_config "16-bit, Channel swap ON" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_swap_channels=1 ngt_alaw_shift=8"

# Test 15: Without channel swap
test_config "16-bit, Channel swap OFF" \
    "ngt_autostart=1 ngt_log_first=20 ngt_resolution=0x01 ngt_swap_channels=0 ngt_alaw_shift=8"

echo "=============================================="
echo "TESTING COMPLETE"
echo "=============================================="
echo ""
echo "If D5 pattern was found in any test, that configuration"
echo "is the correct one for your FPGA!"
echo ""
echo "To manually test a configuration:"
echo "  sudo rmmod designware-i2s-NGT"
echo "  sudo modprobe designware_i2s_NGT ngt_autostart=1 ngt_log_first=50 <params>"
echo "  dmesg | grep D5"
echo ""
