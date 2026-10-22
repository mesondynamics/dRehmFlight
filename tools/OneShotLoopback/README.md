# Teensy 4.0 OneShot125 回读仪表

这是独立台架仪表，会自动在数字引脚 1、2、3 上循环输出真实的 120–250 µs 电调信号；不可作为飞行固件使用。

使用前断开电调动力电池，并拔掉电调连接到 Teensy 1、2、3 的三根信号线。保持 USB 连接，用三根杜邦线连接板上数字引脚：1→14、2→15、3→16。不要将 VCC、CUR 或供电脚接入回读引脚。

```sh
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-oneshot-loopback-build tools/OneShotLoopback
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:110000 --build-path /tmp/drehmflight-oneshot-loopback-build tools/OneShotLoopback
```

上传端口会变化，使用 `arduino-cli board list` 查询当前值。USB 诊断行以 `TEST:1` 开头，`E2/E3/E4` 是各输出目标脉宽，`W2/W3/W4` 是在输入 14/15/16 回读的最小/最大脉宽，`P` 是周期，`N` 是脉冲计数；单位为 µs。同时观测相邻输入 13 和 17，辅助发现排针位置偏移。`4294967295` 与计数 0 表示没有有效回读，不能视为有效脉宽。

输出函数从当前飞行 sketch 的 `commandMotors()` 复制，仪表按约 2 kHz 调用。回读中断和 µs 计时分辨率会带来约 1 µs 差异。该方法检查数字输出及回读连线，不测模拟幅度/振铃、不证明电调接口末端导通，也不验证电调协议支持。

测试完成后，先重新下载正常的 `Versions/dRehmFlight_Teensy_BETA_1.3` 固件并确认串口恢复 `LOOP_US` 等正常诊断、SA=2000、ARM=0、三路电机命令为120。再拔掉回读跳线并接回电调信号，之后才可进行电调上电检查。

本次回读记录见 [motor-bench-verification.json](../../docs/motor-bench-verification.json)。引脚位置以 [PJRC Teensy 4.0 引脚图](https://www.pjrc.com/teensy/pinout.html)为准。
