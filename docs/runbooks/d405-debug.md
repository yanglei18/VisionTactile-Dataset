# D405 调试快速入口

固定设备序列号：`255323071742`。

```bash
lsusb -d 8086:0b5b
lsusb -t
journalctl -k -b --no-pager | rg -i 'realsense|8086|0b5b|uvc|usb 2-2|reset|disconnect'
```

调试期间保持 D405 主机直连；固件只读取，不在默认流程中升级。
