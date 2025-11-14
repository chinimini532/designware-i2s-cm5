# Building the NGT DesignWare I²S Driver

This driver can be built either natively on the Raspberry Pi CM5 or via a
cross-compiler. For most cases, building directly on the CM5 is the simplest.

---

## 1. Native Build on CM5 (Recommended)

Copy the `src/` directory to your CM5 or clone this repo directly on the board.

Then run:

```bash
cd src
make -C /lib/modules/$(uname -r)/build M=$PWD modules
```

This will produce a module file:

```bash
dwc-i2s-ngt.ko
```

Load the module:

```bash
sudo insmod dwc-i2s-ngt.ko
```

Unload:

```bash
sudo rmmod dwc-i2s-ngt
```

Check logs:

```bash
dmesg | grep i2s
```


##Out of the tree build on Host(Cross-Compilation)

A full cross-compilation setup is kept in a separate repository
(cm5-cross-compile-setup).
If your host is already configured with the toolchain:

```bash
cd src
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
     -C /path/to/kernel/source M=$PWD modules
```

##Device Tree Overlay 

Build the overlay:

```bash
dtc -@ -I dts -O dtb ngt-i2s-overlay.dts -o ngt-i2s-overlay.dtbo
```

Load it by adding this to **/boot/firmware/config.txt**:
```bash
dtoverlay=ngt-i2s-overlay
```

Reboot and check:

```bash
dmesg | grep ngt
```


