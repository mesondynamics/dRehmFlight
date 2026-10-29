# macOS 开发环境

本机验证记录（2026-10-15）：Apple Silicon Mac、macOS 26.7.1、Arduino IDE 2.3.10、Arduino CLI 1.5.1、Teensyduino Boards Manager core 1.62.0、Teensy 4.0、CRSFforArduino 2025.12.11。

## 已配置项目

- Arduino IDE Boards Manager 已添加 PJRC URL：`https://www.pjrc.com/teensy/package_teensy_index.json`，安装了 Teensy 1.62.0。
- CRSFforArduino 安装在 `~/Documents/Arduino/libraries/CRSFforArduino`。重装此库后，从仓库根目录运行：

  ```sh
  python3 tools/configure_crsf_realtime.py "$HOME/Documents/Arduino/libraries/CRSFforArduino"
  ```

  这会关闭本固件不使用的 CRSF 下行遥测，避免接收库等待 UART 发完数据；固件会在编译时检查该设置。
- `arduino-cli` 已用 Homebrew 安装并加入 PATH。板卡包数据与 Arduino IDE 共用 `~/Library/Arduino15`，用户库在 `~/Documents/Arduino/libraries`。
- Teensy USB 驱动由 macOS 自带，无需安装 Linux udev 规则。接入的 Teensy 4.0 会显示为 Teensy 上传设备和 `/dev/cu.usbmodem...` 串口。

## 打开工程

用 Arduino IDE 打开整个 sketch 文件夹：
`Versions/dRehmFlight_Teensy_BETA_1.3`

在板卡选择器中选择 **Teensy 4.0**。插拔或重新连接设备后，先运行 `arduino-cli board list` 查看当前 Teensy 端口；`usb:1114000` 和 `/dev/cu.usbmodem160607901` 是本次连接得到的值，之后可能变化。

## 编译和下载

```sh
arduino-cli compile --fqbn teensy:avr:teensy40 Versions/dRehmFlight_Teensy_BETA_1.3
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:1114000 Versions/dRehmFlight_Teensy_BETA_1.3
```

也可在 IDE 使用 Verify 和 Upload。若上传器等待设备，按一下 Teensy 4.0 板上的 Program 按钮，再重试 Upload。

## 串口调试

Arduino IDE 的 Serial Monitor 和 Serial Plotter 已在本机验证可用。Serial Plotter 可按行解析固件输出的 `CH1:... CH2:... GyroX:...` 数据；可通过图例开关选择要看的通道。当前 USB 串口速度为 500000 baud，诊断输出 20 Hz。ELRS 接收机关闭时，通道显示为 0 且 `LINK:0 FS:1`，表示失联保护状态。

本次还在 IDE 中对该工程执行了 Verify 和 Upload，IDE 显示 `Done compiling`、`Done uploading`，随后串口持续输出诊断行。当前遥控器关闭，串口显示 `LINK:0 FS:1 READY:0 ARM:0`，三个电机命令为停机值，舵机命令居中。此前确认电机和舵机控制信号线均未接入，因此此次下载未驱动执行器。

稍后打开 TX16S 后，已收到 ELRS/CRSF 通道帧：CH1=1500、CH2=1502、CH3=1000、CH4=1503、CH5=2000、CH6=1496，状态为 `LINK:1 FS:0 READY:1 ARM:0`。三路电机输出仍为 120 µs；舵机命令为 S1=89（约 1492 µs），与 CH6 略低于中位相符。遥控有效时 `LOOP_US=500`、`LOOP_MAX_US=500`、`WORK_MAX_US=347`、`OVERRUN=0`。执行器信号线保持断开。

按台架安全条件（CH5=2000、油门最低、舵机信号线断开）完成 S1 全行程检查：CH6=1000 时 `S1=0, S1_US=512`，CH6=2000 时 `S1=180, S1_US=2496`；回中 CH6=1496 时 `S1=89, S1_US=1492`。脉宽是按 PWMServo 配置估算，未用示波器测量。随后将 `fullRangeBenchTest` 恢复为 `false`，重新编译并下载；串口确认 `FULLTEST:0`、`LINK:1 FS:0 READY:1 ARM:0`、`CH5:2000`、`CH3:1000`、`ARM:0`，三路电机命令仍是 120 µs。下载后 `LOOP_US=500`、`LOOP_MAX_US=500`、`WORK_MAX_US=336`、`OVERRUN=0`。当前固件为小行程模式，执行器信号线继续保持断开。

随后按用户确认，将 CH6/S1 台架范围和飞行偏航闭环范围均扩大到 270° 舵机轴相对中位约 ±60°，即 `yawServoLimitDeg=40`，PWMServo 逻辑 50–130；`fullRangeBenchTest` 仍为 `false`。以上“小行程”是此前版本的验证记录，当前采用此次 ±60° 配置。混控、限幅与解锁主机检查通过，Teensy 4.0 编译及下载成功（本次上传端口 `usb:110000`）。用户确认拆桨、电调主电源断开、S1 回中后下载；下载后 6 秒捕获 121 条诊断，CH5=2000、CH6=1503、`LINK:1 FS:0 READY:1 ARM:0 FULLTEST:0`、`S1:90 S1_US:1504`，三个电机输出均为 120 µs；`LOOP_US=500`、`LOOP_MAX_US=500`、`WORK_MAX_US=325`、`OVERRUN=0`。扩大后的物理角度与机构止挡尚未实测。

Teensy 4.0 在 Arduino IDE 中不支持源码断点调试；串口监视器、绘图器和仓库中的环路计时字段用于本项目调试。

2026-10-15 随后完成当前 MPU6050 安装姿态下的静止水平标定并将六个固定偏置写入正常固件，标定数据与验证结果见 [imu-calibration.json](imu-calibration.json)。在用户确认电调动力电池断开、机架水平静止的条件下采样 12,000 次，临时标定程序无电机脉冲或舵机 PWM；已恢复并下载正常固件，启动时不重新执行标定。刷写后 10 秒取得 201 条诊断，后半段横滚均值 -0.012°、俯仰均值 0.007°，三轴陀螺仪均值分别约 -0.003、-0.013、0.004 °/s。链路有效且 `ARM=0`，三路电机命令均为 120 µs，`LOOP_US=500`、`LOOP_MAX_US=500`、`WORK_MAX_US=325`、`OVERRUN=0`。这是静止状态验证，电调动力上电、实际电机转向与闭环方向尚待验收。
