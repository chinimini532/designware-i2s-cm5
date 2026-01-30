
# designware-i2s-ngt

This repository contains experimental work on a customized Synopsys DesignWare I²S
driver for the Raspberry Pi Compute Module 5 (CM5) using the RP1 I/O subsystem.

The main goals of this project are:

- Use the DesignWare I²S IP block with RP1 on CM5
- Add an hrtimer-based poller path for bring-up and debug
- Log FIFO activity (TX/RX samples, ISR flags) during operation
- Support simple RX→TX loopback tests without a full ALSA userspace stack
- Keep the existing DesignWare PCM (PIO) glue mostly unchanged

This code is **experimental** and intended for learning, bring-up, and internal
testing, not for production audio use.

---

## Repository Layout

```text
src/    - driver sources (modified DesignWare I²S and PCM, plus Makefile)
dts/    - device-tree snippets / overlays for the I²S block (to be added)
docs/   - design notes, register info, and test documentation (to be added)
logs/   - dmesg traces, register dumps, and poller output (to be added)

```

## High-Level Overview

This driver is based on the upstream ALSA SoC Synopsys DesignWare I²S driver and
extended with:

-An optional poller mode using hrtimer to periodically check and service
TX/RX FIFOs

-FIFO logging to observe TX and RX samples and ISR flags for debugging

-Optional RX→TX loopback to test the I²S path without external audio sources

-RP1-specific configuration for DMA/register mappings on CM5

The intention is to make it easier to bring up and debug I²S on CM5 with RP1,
especially when normal DMA/interrupt-driven audio paths are not fully stable yet.

---

## Build Notes

This driver was built as a kernel module for Raspberry Pi CM5.

It can be built in two ways:

-Natively on the CM5 (recommended for simplicity), or

-With a cross-compiler targeting the CM5 kernel

This repository does not try to document the full cross-compilation or CM5
kernel environment. For that, a separate CM5 setup / environment repo can be used.

Here, the focus is only on:

-The modified I²S driver

-The associated DTS snippets

-The logs and notes from bring-up

A simple out-of-tree build flow (example) will be documented later in docs/.

---

## Device Tree (DTS)

```text
compatible = "netgenetech,dw-i2s-ngt";
```
## License
The original DesignWare I²S and PCM code is licensed under the **GNU General
Public License version 2 (GPLv2)**, and this modified version remains under the
same license.

---






=======
# designware-i2s-cm5
