# DesignWare I2S Driver for Raspberry Pi 5 / CM5

**Custom Synopsys DesignWare I2S Driver with RP1 Support**

A custom I2S kernel driver for Raspberry Pi 5 and Compute Module 5 (CM5) featuring real-time FIFO monitoring, configurable clock rates, and FPGA loopback testing capabilities.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Hardware Requirements](#hardware-requirements)
4. [Technical Specifications](#technical-specifications)
5. [Installation](#installation)
6. [Usage](#usage)
7. [Test Results](#test-results)
8. [I2S Protocol Analysis](#i2s-protocol-analysis)
9. [Customization Guide](#customization-guide)
10. [Troubleshooting](#troubleshooting)
11. [Repository Structure](#repository-structure)
12. [License](#license)

---

## Overview

This project provides a **modified Synopsys DesignWare I2S driver** specifically optimized for the **Raspberry Pi 5** and **Compute Module 5 (CM5)** platforms using the **RP1 I/O controller**.

### Why This Driver?

The standard Linux I2S driver lacks debugging capabilities essential for hardware bring-up and FPGA integration. This driver adds:

- **Real-time FIFO monitoring** via high-resolution timer polling
- **Configurable BCLK rates** from 6.4 kHz to 50 MHz
- **Direct loopback testing** without ALSA userspace dependencies
- **Comprehensive logging** of TX/RX samples and interrupt status

### Target Applications

- **Hardware Bring-up** - Debug I2S timing issues with detailed FIFO logging
- **FPGA Integration** - Test FPGA-based audio interfaces with Pi as master
- **Embedded Audio** - Implement custom audio protocols (A-law, u-law, etc.)
- **Education** - Learn I2S protocol timing and behavior

---

## Features

### Core Features

| Feature | Description | Status |
|---------|-------------|--------|
| RP1 I2S Support | Native support for Pi 5's RP1 I/O controller | Complete |
| hrtimer Polling | High-resolution timer for precise FIFO servicing | Complete |
| FIFO Logging | Real-time TX/RX sample logging with ISR flags | Complete |
| Loopback Mode | RX to TX loopback without external hardware | Complete |
| Clock Control | Runtime configurable sample rates (100 Hz - 384 kHz) | Complete |
| Frame Formats | Standard I2S and Left-Justified support | Complete |

### Module Parameters

```bash
sudo insmod dwc-i2s-ngt.ko \
    ngt_sample_rate=8000 \
    ngt_ccr=0x10 \
    ngt_frame_offset=1 \
    ngt_use_poll=1 \
    ngt_poll_us=100 \
    ngt_autostart=1 \
    ngt_log_every=100 \
    ngt_log_first=8
```

---

## Hardware Requirements

### Supported Platforms

| Platform | Tested | Notes |
|----------|--------|-------|
| Raspberry Pi 5 | Yes | Primary development platform |
| Compute Module 5 | Yes | Full support with IO board |
| Compute Module 4 | Partial | Different I2S base address |

### Pin Configuration

| GPIO | Function | Direction | Description |
|------|----------|-----------|-------------|
| GPIO 18 | PCM_CLK (BCLK) | Output | Bit clock |
| GPIO 19 | PCM_FS (LRCLK) | Output | Frame sync / Word select |
| GPIO 20 | PCM_DIN (SDI) | Input | Serial data input |
| GPIO 21 | PCM_DOUT (SDO) | Output | Serial data output |

### Hardware Setup Diagram

```
    Raspberry Pi 5                      FPGA
    +--------------+                +--------------+
    |              |                |              |
    | GPIO18 BCLK  |--------------->| BCLK_IN      |
    | GPIO19 LRCLK |--------------->| LRCLK_IN     |
    | GPIO21 SDO   |--------------->| SDI          |
    | GPIO20 SDI   |<---------------| SDO          |
    | GND          |----------------| GND          |
    |              |                |              |
    +--------------+                +--------------+

    Mode: Pi = Master, FPGA = Slave
    Pi generates BCLK and LRCLK
```

---

## Technical Specifications

### I2S Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| I2S Base Address | 0x1f000a0000 | RP1 I2S controller |
| Source Clock | 50 MHz | From RP1 xosc |
| Data Width | 32 bits | Per channel |
| Channels | 2 (Stereo) | Left + Right |
| Frame Size | 64 bits | 32L + 32R |
| Byte Order | MSB First | Standard I2S |

### Clock Configuration

| Sample Rate | BCLK Frequency | Formula |
|-------------|----------------|---------|
| 100 Hz | 6.4 kHz | 100 x 64 |
| 1,000 Hz | 64 kHz | 1,000 x 64 |
| 8,000 Hz | 512 kHz | 8,000 x 64 |
| 32,000 Hz | 2.048 MHz | 32,000 x 64 |
| 44,100 Hz | 2.8224 MHz | 44,100 x 64 |
| 48,000 Hz | 3.072 MHz | 48,000 x 64 |

**Formula:** `BCLK = Sample_Rate x Bits_Per_Channel x Num_Channels`

**For 32-bit stereo:** `BCLK = Sample_Rate x 64`

### Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| IER | 0x000 | I2S Enable Register |
| IRER | 0x004 | I2S Receiver Enable |
| ITER | 0x008 | I2S Transmitter Enable |
| CER | 0x00C | Clock Enable Register |
| CCR | 0x010 | Clock Configuration Register |
| RXFFR | 0x014 | Receiver FIFO Reset |
| TXFFR | 0x018 | Transmitter FIFO Reset |
| LTHR(0) | 0x020 | Left TX Holding Register |
| RTHR(0) | 0x024 | Right TX Holding Register |

---

## Installation

### Prerequisites

On your Ubuntu host machine:

```bash
# Install cross-compilation tools
sudo apt update
sudo apt install gcc-aarch64-linux-gnu build-essential

# You need Raspberry Pi kernel source
# Clone or download from: https://github.com/raspberrypi/linux
```

### Building the Driver (Cross-Compilation)

```bash
# Set up environment variables
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export KDIR=/path/to/raspberry-pi-kernel-source

# Navigate to driver source
cd designware-i2s-ngt/src

# Build
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $KDIR M=$(pwd) modules

# Copy to Pi
scp dwc-i2s-ngt.ko pi@<PI_IP>:/home/pi/
```

### Device Tree Overlay

On the Raspberry Pi:

```bash
# Copy overlay to boot partition
sudo cp ngt-i2s.dtbo /boot/firmware/overlays/

# Edit config.txt
sudo nano /boot/firmware/config.txt

# Add this line:
dtoverlay=ngt-i2s

# Reboot
sudo reboot
```

### Loading the Module

```bash
# Basic load
sudo insmod dwc-i2s-ngt.ko

# With custom parameters
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=8000 ngt_autostart=1

# Verify
lsmod | grep i2s
dmesg | tail -30
```

---

## Usage

### Basic Loopback Test

```bash
# Load driver with 8 kHz sample rate
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=8000

# Check output
dmesg | grep NGT
```

Expected output:
```
NGT[2] TX=0xd5a5d5a5 | RX=0xd5a5d5a5 | DIRECT MATCH (no shift) | exact=1/1 (100%)
```

### Testing Different Clock Speeds

```bash
# Slow clock for oscilloscope debugging
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=100   # 6.4 kHz BCLK

# Standard telephony rate
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=8000  # 512 kHz BCLK

# CD quality audio
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=44100 # 2.82 MHz BCLK
```

### Monitoring in Real-Time

```bash
# Watch dmesg output
sudo dmesg -w | grep NGT

# Check clock status
cat /sys/kernel/debug/clk/clk_summary | grep clk_i2s
```

---

## Test Results

### Clock Configuration Results

| Target Rate | Requested BCLK | Actual BCLK | Status |
|-------------|----------------|-------------|--------|
| 100 Hz | 6,400 Hz | 6,400 Hz | Success |
| 1,000 Hz | 64,000 Hz | 64,000 Hz | Success |
| 8,000 Hz | 512,000 Hz | 512,000 Hz | Success |
| 32,000 Hz | 2,048,000 Hz | 2,048,000 Hz | Success |

### Data Integrity Results

| BCLK Speed | TX Pattern | RX Pattern | Shift | Match Rate |
|------------|------------|------------|-------|------------|
| 512 kHz | 0xD5A5D5A5 | 0xD5A5D5A5 | None | 100% |
| 50 MHz | 0xD5A5D5A5 | 0x6AD2EAD2 | 1-bit right | 100% (corrected) |

### Key Finding

At proper audio clock rates (512 kHz BCLK for 8 kHz sample rate), the I2S interface operates correctly with NO bit shifting required.

---

## I2S Protocol Analysis

### Standard I2S Timing

The I2S protocol specifies a 1-bit delay between the word select (LRCLK) transition and the first data bit (MSB).

```
BCLK:   _|~|_|~|_|~|_|~|_|~|_|~|_|~|_|~|_|~|_|~|_|~|_|~|

LRCLK:  ________________|~~~~~~~~~~~~~~~~~|________________
                        ^
                        | WS transition

DATA:   -------------------|D31|D30|D29|D28|...
                        ^  ^
                        |  |
                        |  +-- MSB appears here (1 BCLK after WS)
                        +---- WS changes here
```

### Left-Justified vs Standard I2S

| Format | Data Alignment | Delay After WS |
|--------|----------------|----------------|
| Standard I2S | MSB aligned, 1-bit delay | 1 BCLK cycle |
| Left-Justified | MSB aligned, no delay | 0 BCLK cycles |

### Bit Shift Analysis

At 512 kHz BCLK (Normal Rate):
```
TX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
RX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
Result: PERFECT MATCH - No bit shift!
```

At 50 MHz BCLK (Too Fast):
```
TX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
RX: 0x6AD2EAD2 = 0110 1010 1101 0010 1110 1010 1101 0010
Result: Right-shifted by 1 bit (timing artifact)
```

---

## Customization Guide

### Module Parameter Reference

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| ngt_sample_rate | uint | 8000 | 100-384000 | Sample rate in Hz |
| ngt_ccr | uint | 0x10 | 0x00-0x18 | Clock config register |
| ngt_frame_offset | uint | 0 | 0-1 | 0=Left-justified, 1=I2S |
| ngt_use_poll | bool | true | - | Enable hrtimer polling |
| ngt_poll_us | uint | 100 | 10-10000 | Poll interval (us) |
| ngt_autostart | bool | true | - | Auto-start on load |
| ngt_log_every | uint | 100 | 1-10000 | Log frequency |
| ngt_log_first | uint | 8 | 0-1000 | Initial samples to log |
| ngt_fifo_burst | uint | 8 | 1-16 | FIFO burst size |
| ngt_tdm_slots | uint | 1 | 1-4 | TDM slots per channel |
| ngt_slot_width | uint | 32 | 16-32 | Bits per slot |

### Changing TX Data Pattern

Edit `dwc-i2s-ngt.c` around line 750:

```c
// Change TX pattern
tx_data = 0xD5A5D5A5;  // Your custom pattern

// For A-law audio (8-bit in upper byte):
tx_data = (alaw_sample << 24) | (alaw_sample << 8);

// For 16-bit audio:
tx_data = (left_sample << 16) | right_sample;
```

### Adapting for Different FPGAs

Example I2S Slave Receiver (Verilog):

```verilog
module i2s_slave_rx (
    input  wire        bclk,
    input  wire        lrclk,
    input  wire        sdi,
    output reg  [31:0] left_data,
    output reg  [31:0] right_data,
    output reg         data_valid
);
    // Your implementation here
    // Remember: Standard I2S has 1-BCLK delay after LRCLK edge
endmodule
```

---

## Troubleshooting

### Module doesn't load - "Unknown symbol"

Cause: Kernel version mismatch

Solution:
```bash
uname -r
make clean
make KDIR=/lib/modules/$(uname -r)/build
```

### No output in dmesg

Cause: Device tree overlay not loaded

Solution:
```bash
# Check overlay is active
dtoverlay -l

# Verify compatible string
cat /proc/device-tree/axi/pcie@1000120000/rp1/i2s@a0000/compatible
# Should show: netgenetech,dw-i2s-ngt
```

### Clock stuck at 50 MHz

Cause: Clock rate change not supported

Solution: Make sure you're using the ngt-i2s overlay that enables clock control.

### RX data all zeros

Cause: No loopback connection or FPGA not responding

Solution:
1. Check physical connections
2. Verify FPGA is powered and programmed
3. Check clock signals with oscilloscope

---

## Repository Structure

```
designware-i2s-ngt/
|-- README.md
|-- LICENSE
|-- src/
|   |-- dwc-i2s-ngt.c
|   |-- dw_pcm.c
|   |-- local.h
|   +-- Makefile
|-- overlays/
|   |-- ngt-i2s-overlay.dts
|   +-- ngt-i2s.dtbo
|-- docs/
|   +-- images/
+-- logs/
```

---

## License

This project is licensed under the **GNU General Public License v2.0**.

The original DesignWare I2S driver is copyright ST Microelectronics and licensed under GPL v2.

---

## Acknowledgments

- **Synopsys** - Original DesignWare I2S IP design
- **ST Microelectronics** - Original Linux driver implementation
- **Raspberry Pi Foundation** - RP1 documentation and kernel support
- **Linux ALSA Community** - Sound subsystem framework

---

## Author

Developed for FPGA-based audio interface testing on Raspberry Pi 5 / CM5.

For questions or contributions, please open an issue on GitHub.
