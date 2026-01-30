## Device Tree Overlay

The `dts/ngt-i2s-overlay.dts` file is a simple DT overlay that retargets the
RP1 I²S instance at `i2s@a0000` to the custom NGT DesignWare I²S driver by
setting:

```dts
compatible = "netgenetech,dw-i2s-ngt";
status = "okay";
```

It uses the base node path:
```text
/axi/pcie@1000120000/rp1/i2s@a0000
```
