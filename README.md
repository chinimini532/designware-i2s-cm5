<![CDATA[<div align="center">

# 🎵 DesignWare I²S Driver for Raspberry Pi 5 / CM5

### Custom Synopsys DesignWare I²S Driver with RP1 Support

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205%20%7C%20CM5-red.svg)](https://www.raspberrypi.com/)
[![Kernel](https://img.shields.io/badge/Kernel-6.x-green.svg)](https://github.com/raspberrypi/linux)
[![Status](https://img.shields.io/badge/Status-Experimental-orange.svg)]()

<br>

**A custom I²S kernel driver for Raspberry Pi 5 and Compute Module 5 (CM5) featuring real-time FIFO monitoring, configurable clock rates, and FPGA loopback testing capabilities.**

<br>

[Features](#-features) •
[Quick Start](#-quick-start) •
[Technical Details](#-technical-details) •
[Test Results](#-test-results) •
[Customization](#-customization-guide) •
[Contributing](#-contributing)

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Technical Specifications](#-technical-specifications)
- [Installation](#-installation)
- [Usage](#-usage)
- [Test Results](#-test-results)
- [I²S Protocol Analysis](#-i2s-protocol-analysis)
- [Customization Guide](#-customization-guide)
- [Troubleshooting](#-troubleshooting)
- [Repository Structure](#-repository-structure)
- [Research & Publications](#-research--publications)
- [License](#-license)
- [Acknowledgments](#-acknowledgments)

---

## 🔍 Overview

This project provides a **modified Synopsys DesignWare I²S driver** specifically optimized for the **Raspberry Pi 5** and **Compute Module 5 (CM5)** platforms using the **RP1 I/O controller**.

### Why This Driver?

The standard Linux I²S driver lacks debugging capabilities essential for hardware bring-up and FPGA integration. This driver adds:

- **Real-time FIFO monitoring** via high-resolution timer polling
- **Configurable BCLK rates** from 6.4 kHz to 50 MHz
- **Direct loopback testing** without ALSA userspace dependencies
- **Comprehensive logging** of TX/RX samples and interrupt status

### Target Applications

| Application | Description |
|-------------|-------------|
| 🔧 **Hardware Bring-up** | Debug I²S timing issues with detailed FIFO logging |
| 🎛️ **FPGA Integration** | Test FPGA-based audio interfaces with Pi as master |
| 📡 **Embedded Audio** | Implement custom audio protocols (A-law, μ-law, etc.) |
| 📚 **Education** | Learn I²S protocol timing and behavior |

---

## ✨ Features

### Core Features

| Feature | Description | Status |
|---------|-------------|--------|
| 🎯 **RP1 I²S Support** | Native support for Pi 5's RP1 I/O controller | ✅ Complete |
| ⏱️ **hrtimer Polling** | High-resolution timer for precise FIFO servicing | ✅ Complete |
| 📊 **FIFO Logging** | Real-time TX/RX sample logging with ISR flags | ✅ Complete |
| 🔄 **Loopback Mode** | RX→TX loopback without external hardware | ✅ Complete |
| ⚡ **Clock Control** | Runtime configurable sample rates (100 Hz - 384 kHz) | ✅ Complete |
| 🎚️ **Frame Formats** | Standard I²S and Left-Justified support | ✅ Complete |

### Module Parameters

```bash
# All configurable via insmod or modprobe
sudo insmod dwc-i2s-ngt.ko \
    ngt_sample_rate=8000 \    # Sample rate in Hz (controls BCLK)
    ngt_ccr=0x10 \            # Clock configuration register
    ngt_frame_offset=1 \      # 0=Left-Justified, 1=Standard I²S
    ngt_use_poll=1 \          # Enable hrtimer polling
    ngt_poll_us=100 \         # Poll interval in microseconds
    ngt_autostart=1 \         # Auto-start on module load
    ngt_log_every=100 \       # Log every N polls
    ngt_log_first=8           # Log first N samples
```

---

## 🔧 Hardware Requirements

### Supported Platforms

| Platform | Tested | Notes |
|----------|--------|-------|
| Raspberry Pi 5 | ✅ Yes | Primary development platform |
| Compute Module 5 | ✅ Yes | Full support with IO board |
| Compute Module 4 | ⚠️ Partial | Different I²S base address |

### Pin Configuration

| GPIO | Function | Direction | Description |
|------|----------|-----------|-------------|
| GPIO 18 | PCM_CLK (BCLK) | Output | Bit clock |
| GPIO 19 | PCM_FS (LRCLK) | Output | Frame sync / Word select |
| GPIO 20 | PCM_DIN (SDI) | Input | Serial data input |
| GPIO 21 | PCM_DOUT (SDO) | Output | Serial data output |

### Hardware Setup Diagram

```
┌─────────────────┐                    ┌─────────────────┐
│  Raspberry Pi 5 │                    │      FPGA       │
│                 │                    │                 │
│  GPIO18 (BCLK) ─┼──────────────────►─┼─ BCLK_IN       │
│  GPIO19 (LRCLK)─┼──────────────────►─┼─ LRCLK_IN      │
│  GPIO21 (SDO)  ─┼──────────────────►─┼─ SDI           │
│  GPIO20 (SDI)  ─┼─◄────────────────┼─ SDO           │
│                 │                    │                 │
│     GND        ─┼──────────────────┼─ GND           │
└─────────────────┘                    └─────────────────┘
        │
        │ Master Mode: Pi generates BCLK and LRCLK
        │ FPGA is slave, responds to Pi's clocks
```

---

## 📐 Technical Specifications

### I²S Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| **I²S Base Address** | `0x1f000a0000` | RP1 I²S controller |
| **Source Clock** | 50 MHz | From RP1 xosc |
| **Data Width** | 32 bits | Per channel |
| **Channels** | 2 (Stereo) | Left + Right |
| **Frame Size** | 64 bits | 32L + 32R |
| **Byte Order** | MSB First | Standard I²S |

### Clock Configuration

| Sample Rate | BCLK Frequency | Formula |
|-------------|----------------|---------|
| 100 Hz | 6.4 kHz | 100 × 64 |
| 1,000 Hz | 64 kHz | 1,000 × 64 |
| 8,000 Hz | 512 kHz | 8,000 × 64 |
| 44,100 Hz | 2.8224 MHz | 44,100 × 64 |
| 48,000 Hz | 3.072 MHz | 48,000 × 64 |
| 96,000 Hz | 6.144 MHz | 96,000 × 64 |
| 192,000 Hz | 12.288 MHz | 192,000 × 64 |

> **Formula:** `BCLK = Sample_Rate × Bits_Per_Channel × Num_Channels`
> **For 32-bit stereo:** `BCLK = Sample_Rate × 64`

### Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| IER | 0x000 | I²S Enable Register |
| IRER | 0x004 | I²S Receiver Enable |
| ITER | 0x008 | I²S Transmitter Enable |
| CER | 0x00C | Clock Enable Register |
| CCR | 0x010 | Clock Configuration Register |
| RXFFR | 0x014 | Receiver FIFO Reset |
| TXFFR | 0x018 | Transmitter FIFO Reset |
| LTHR(0) | 0x020 | Left TX Holding Register |
| RTHR(0) | 0x024 | Right TX Holding Register |
| LRBR(0) | 0x020 | Left RX Buffer Register |
| RRBR(0) | 0x024 | Right RX Buffer Register |

---

## 🚀 Installation

### Prerequisites

```bash
# Install kernel headers
sudo apt update
sudo apt install raspberrypi-kernel-headers build-essential

# For cross-compilation (on host machine)
sudo apt install gcc-aarch64-linux-gnu
```

### Building the Driver

#### Option 1: Native Build (On Pi 5)

```bash
git clone https://github.com/yourusername/designware-i2s-ngt.git
cd designware-i2s-ngt/src
make
```

#### Option 2: Cross-Compilation (Recommended)

```bash
# Set up environment
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export KDIR=/path/to/raspberry-pi-kernel-source

# Build
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $KDIR M=$(pwd) modules

# Copy to Pi
scp dwc-i2s-ngt.ko pi@raspberrypi:/home/pi/
```

### Device Tree Overlay

```bash
# Copy overlay to boot partition
sudo cp overlays/ngt-i2s.dtbo /boot/firmware/overlays/

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

## 📖 Usage

### Basic Loopback Test

```bash
# 1. Load driver with 8 kHz sample rate
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=8000

# 2. Check output
dmesg | grep NGT

# 3. Expected output:
# NGT[2] TX=0xd5a5d5a5 | RX=0xd5a5d5a5 | DIRECT MATCH (no shift) | exact=1/1 (100%)
```

### Testing Different Clock Speeds

```bash
# Slow clock for oscilloscope debugging
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=100   # 6.4 kHz BCLK

# Standard audio rate
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=8000  # 512 kHz BCLK

# High quality audio
sudo insmod dwc-i2s-ngt.ko ngt_sample_rate=48000 # 3.072 MHz BCLK
```

### Monitoring in Real-Time

```bash
# Watch dmesg output
sudo dmesg -w | grep NGT

# Check clock status
cat /sys/kernel/debug/clk/clk_summary | grep clk_i2s
```

---

## 📊 Test Results

### Clock Configuration Results

| Target Rate | Requested BCLK | Actual BCLK | Status |
|-------------|----------------|-------------|--------|
| 100 Hz | 6,400 Hz | 6,400 Hz | ✅ Success |
| 1,000 Hz | 64,000 Hz | 64,000 Hz | ✅ Success |
| 8,000 Hz | 512,000 Hz | 512,000 Hz | ✅ Success |
| 48,000 Hz | 3,072,000 Hz | 3,072,000 Hz | ✅ Success |

### Data Integrity Results

| BCLK Speed | TX Pattern | RX Pattern | Shift | Match Rate |
|------------|------------|------------|-------|------------|
| 512 kHz | 0xD5A5D5A5 | 0xD5A5D5A5 | None | **100%** |
| 50 MHz | 0xD5A5D5A5 | 0x6AD2EAD2 | 1-bit right | 100% (with correction) |

### Screenshots

<!-- Add your actual screenshots here -->

#### Oscilloscope Captures

| Description | Image |
|-------------|-------|
| BCLK at 512 kHz | ![BCLK Waveform](docs/images/bclk_512khz.png) |
| LRCLK at 8 kHz | ![LRCLK Waveform](docs/images/lrclk_8khz.png) |
| Data Signal | ![Data Waveform](docs/images/data_signal.png) |
| Full I²S Timing | ![Full Timing](docs/images/full_timing.png) |

> 📸 **To add your images:** Place them in `docs/images/` and update the paths above.

#### dmesg Output

| Description | Image |
|-------------|-------|
| Module Load | ![dmesg output](docs/images/dmesg_load.png) |
| Clock Config | ![Clock config](docs/images/dmesg_clock.png) |
| Data Match | ![Data match](docs/images/dmesg_match.png) |

---

## 🔬 I²S Protocol Analysis

### Standard I²S Timing

The I²S (Inter-IC Sound) protocol, defined by Philips (now NXP), specifies a **1-bit delay** between the word select (LRCLK) transition and the first data bit (MSB).

```
Standard I²S Timing:
                    ┌───────────────────────────────
LRCLK (WS):  ───────┘
                    │
                    │ ← 1 BCLK delay
                    │   ┌─────┬─────┬─────┬─────
DATA:        ───────────┤ MSB │ b30 │ b29 │ ...
                        └─────┴─────┴─────┴─────
                    │   │
                    ├───┤
                    1 BCLK
```

### Left-Justified vs Standard I²S

| Format | Data Alignment | Delay After WS |
|--------|----------------|----------------|
| **Standard I²S** | MSB aligned, 1-bit delay | 1 BCLK cycle |
| **Left-Justified** | MSB aligned, no delay | 0 BCLK cycles |

### Observed Behavior

#### At 512 kHz BCLK (Normal Audio Rate)

```
TX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
RX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
                 ↑
                 PERFECT MATCH - No bit shift!
```

#### At 50 MHz BCLK (Abnormally Fast)

```
TX: 0xD5A5D5A5 = 1101 0101 1010 0101 1101 0101 1010 0101
RX: 0x6AD2EAD2 = 0110 1010 1101 0010 1110 1010 1101 0010
                 ↑
                 Right-shifted by 1 bit (timing artifact)
```

### Key Finding

> **At proper audio clock rates (512 kHz BCLK for 8 kHz sample rate), the I²S interface operates correctly with NO bit shifting required.**

The 1-bit shift observed at 50 MHz is an artifact of operating far outside normal I²S timing specifications, where setup/hold time violations occur in the loopback path.

---

## ⚙️ Customization Guide

### Adapting for Different FPGAs

#### 1. Xilinx FPGA (Spartan/Artix/Kintex)

```verilog
// Example I²S Slave Receiver for Xilinx
module i2s_slave_rx (
    input  wire        bclk,      // Bit clock from Pi
    input  wire        lrclk,     // Word select from Pi
    input  wire        sdi,       // Serial data from Pi
    output reg  [31:0] left_data,
    output reg  [31:0] right_data,
    output reg         data_valid
);
    // Your implementation here
    // Remember: Standard I²S has 1-BCLK delay after LRCLK edge
endmodule
```

#### 2. Intel/Altera FPGA

```verilog
// Similar implementation with Intel-specific primitives
// Use PLL for clock management if needed
```

#### 3. Lattice FPGA

```verilog
// Lattice-specific implementation
// Consider using their I²S IP core
```

### Changing Clock Source

To use a different clock source, modify the device tree overlay:

```dts
// Example: Using Audio PLL instead of xosc
fragment@1 {
    target = <&i2s_clk_producer>;
    __overlay__ {
        // Reference audio PLL for cleaner clock
        clocks = <&rp1_clocks RP1_CLK_AUDIO>;
        clock-names = "i2sclk";
    };
};
```

### Modifying TX Data Pattern

Edit the polling callback in `dwc-i2s-ngt.c`:

```c
// Line ~750: Change TX pattern
tx_data = 0xD5A5D5A5;  // Your custom pattern

// For A-law audio (8-bit in upper byte):
tx_data = (alaw_sample << 24) | (alaw_sample << 8);

// For 16-bit audio:
tx_data = (left_sample << 16) | right_sample;
```

### Adding Custom Processing

```c
// In ngt_poll_cb(), add your processing after RX read:
rx_data = i2s_read_reg(dev->i2s_base, LRBR_LTHR(0));

// Your custom processing:
processed_data = my_audio_filter(rx_data);

// Write processed data to TX:
i2s_write_reg(dev->i2s_base, LRBR_LTHR(0), processed_data);
```

### Module Parameter Reference

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `ngt_sample_rate` | uint | 8000 | 100-384000 | Sample rate in Hz |
| `ngt_ccr` | uint | 0x10 | 0x00-0x18 | Clock config register |
| `ngt_frame_offset` | uint | 0 | 0-1 | 0=Left-justified, 1=I²S |
| `ngt_use_poll` | bool | true | - | Enable hrtimer polling |
| `ngt_poll_us` | uint | 100 | 10-10000 | Poll interval (µs) |
| `ngt_autostart` | bool | true | - | Auto-start on load |
| `ngt_log_every` | uint | 100 | 1-10000 | Log frequency |
| `ngt_log_first` | uint | 8 | 0-1000 | Initial samples to log |
| `ngt_fifo_burst` | uint | 8 | 1-16 | FIFO burst size |
| `ngt_bclk_div` | uint | 0 | 0-65535 | Direct BCLK divider |

---

## 🔧 Troubleshooting

### Common Issues

<details>
<summary><b>❌ Module doesn't load - "Unknown symbol"</b></summary>

**Cause:** Kernel version mismatch

**Solution:**
```bash
# Check kernel version
uname -r

# Rebuild against current kernel
make clean
make KDIR=/lib/modules/$(uname -r)/build
```
</details>

<details>
<summary><b>❌ No output in dmesg</b></summary>

**Cause:** Device tree overlay not loaded or wrong compatible string

**Solution:**
```bash
# Check overlay is active
dtoverlay -l

# Verify compatible string
cat /proc/device-tree/axi/pcie@1000120000/rp1/i2s@a0000/compatible
# Should show: netgenetech,dw-i2s-ngt
```
</details>

<details>
<summary><b>❌ Clock stuck at 50 MHz</b></summary>

**Cause:** Clock rate change not supported by current overlay

**Solution:**
```bash
# Check if clock changed
cat /sys/kernel/debug/clk/clk_summary | grep clk_i2s

# If still 50 MHz, ensure correct overlay is loaded
# The ngt-i2s overlay enables clock rate changes
```
</details>

<details>
<summary><b>❌ RX data all zeros</b></summary>

**Cause:** No loopback connection or FPGA not responding

**Solution:**
1. Check physical connections (GPIO 20 ↔ GPIO 21 for loopback)
2. Verify FPGA is powered and programmed
3. Check clock signals with oscilloscope
</details>

<details>
<summary><b>❌ Data shifted by 1 bit</b></summary>

**Cause:** Normal I²S behavior at high clock rates

**Solution:**
- Use slower sample rate (8 kHz recommended)
- The driver handles this automatically with match detection
</details>

---

## 📁 Repository Structure

```
designware-i2s-ngt/
├── 📄 README.md                 # This file
├── 📄 LICENSE                   # GPL v2 License
├── 📁 src/                      # Driver source code
│   ├── dwc-i2s-ngt.c           # Main driver (modified DesignWare I²S)
│   ├── dw_pcm.c                # PCM/PIO helper
│   ├── local.h                 # Local definitions
│   └── Makefile                # Build configuration
├── 📁 overlays/                 # Device tree overlays
│   ├── ngt-i2s-overlay.dts     # Main overlay source
│   └── ngt-i2s.dtbo            # Compiled overlay
├── 📁 docs/                     # Documentation
│   ├── 📁 images/              # Screenshots and diagrams
│   │   ├── bclk_512khz.png    # BCLK oscilloscope capture
│   │   ├── lrclk_8khz.png     # LRCLK oscilloscope capture
│   │   ├── data_signal.png    # Data line capture
│   │   ├── full_timing.png    # Full I²S timing diagram
│   │   ├── dmesg_load.png     # Module load screenshot
│   │   ├── dmesg_clock.png    # Clock config screenshot
│   │   └── dmesg_match.png    # Data match screenshot
│   ├── I2S_PROTOCOL.md         # I²S protocol explanation
│   ├── RP1_REGISTERS.md        # RP1 I²S register documentation
│   └── FPGA_INTEGRATION.md     # FPGA integration guide
├── 📁 logs/                     # Test logs and traces
│   ├── dmesg_512khz.txt        # dmesg output at 512 kHz
│   └── dmesg_50mhz.txt         # dmesg output at 50 MHz
└── 📁 tools/                    # Helper scripts
    ├── load_driver.sh          # Driver loading script
    └── test_loopback.sh        # Loopback test script
```

---

## 📚 Research & Publications

### Potential Publication Venues

This work could be suitable for publication in:

#### Journals

| Journal | Focus | Impact |
|---------|-------|--------|
| **IEEE Embedded Systems Letters** | Embedded systems, hardware-software co-design | Medium |
| **Journal of Low Power Electronics and Applications (JLPEA)** | Low-power embedded systems | Open Access |
| **Electronics (MDPI)** | General electronics, embedded systems | Open Access |
| **Microprocessors and Microsystems (Elsevier)** | Embedded systems, SoCs | High |

#### Conferences

| Conference | Focus | Deadline |
|------------|-------|----------|
| **IEEE International Symposium on Circuits and Systems (ISCAS)** | Circuits & Systems | ~Oct yearly |
| **Design, Automation & Test in Europe (DATE)** | EDA, Embedded Systems | ~Sept yearly |
| **IEEE International Conference on Electronics, Circuits and Systems (ICECS)** | Electronics | ~July yearly |

### Suggested Research Extensions

To make this work more publishable, consider adding:

1. **Performance Benchmarks**
   - Latency measurements (TX to RX)
   - Jitter analysis of clock signals
   - CPU utilization comparison (polling vs DMA vs interrupt)

2. **Audio Quality Metrics**
   - THD+N (Total Harmonic Distortion + Noise)
   - SNR (Signal-to-Noise Ratio)
   - Channel separation

3. **Comparative Analysis**
   - Compare with BCM2835 I²S (Pi 4 and earlier)
   - Compare with USB audio
   - Compare with other embedded platforms

4. **Real-World Applications**
   - Voice codec implementation (A-law/μ-law)
   - Audio effects processing
   - Multi-channel audio routing

### Citation

If you use this work, please cite:

```bibtex
@misc{designware-i2s-ngt,
  author       = {Your Name},
  title        = {DesignWare I²S Driver for Raspberry Pi 5 with RP1 Support},
  year         = {2025},
  publisher    = {GitHub},
  url          = {https://github.com/yourusername/designware-i2s-ngt}
}
```

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

### How to Contribute

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- Follow Linux kernel coding style
- Use meaningful commit messages
- Add comments for complex logic
- Update documentation for new features

---

## 📄 License

This project is licensed under the **GNU General Public License v2.0** - see the [LICENSE](LICENSE) file for details.

The original DesignWare I²S driver is copyright © ST Microelectronics and licensed under GPL v2.

---

## 🙏 Acknowledgments

- **Synopsys** - Original DesignWare I²S IP design
- **ST Microelectronics** - Original Linux driver implementation
- **Raspberry Pi Foundation** - RP1 documentation and kernel support
- **Linux ALSA Community** - Sound subsystem framework

---

<div align="center">

### ⭐ Star this repo if you found it useful!

**Made with ❤️ for the embedded audio community**

[Report Bug](https://github.com/yourusername/designware-i2s-ngt/issues) •
[Request Feature](https://github.com/yourusername/designware-i2s-ngt/issues) •
[Discussions](https://github.com/yourusername/designware-i2s-ngt/discussions)

</div>
]]>
