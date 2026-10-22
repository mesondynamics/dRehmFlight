# 有限幅和自动停机的电机台架固件

可生成 OneShot125、DShot600 和 DShot300 协议测试镜像。以下为默认 OneShot125 用法；DShot600 的生成、计时与诊断说明见 [DSHOT_BENCH.md](../../docs/DSHOT_BENCH.md)。

仅用于拆桨试转排查，不能用于起飞。生成器复制当前已标定的飞行 sketch 到仓库外的临时目录，并加入三路电机输出限幅及三秒停机锁定。正常飞行 sketch 不加入此配置。

- 有效遥控链路和正常解锁条件仍必须满足。
- 三路运行命令限制在 125–137 µs；原来的 SA 停机和失联保护仍输出 120 µs。
- 解锁后，油门 CH3 首次超过 1040 时开始计时；三秒后停机并锁定。降低油门不能解除锁定，须先将 SA/CH5 回到 2000。
- 串口标记 `BENCH:1 CAP_US:137 TIMEOUT:...`，原始 CH3 仍打印实际遥控读数，不会因输出限幅改变。
- 默认不周期打印；串口发送 `?` 查询一行状态。三秒超时撤销解锁，偏航舵机随之回中，SA回停机才能清除锁定。此行为仅存在于台架镜像。
- 使用校准过的 IMU、三旋翼混控和当前约 ±60° 舵机轴限幅；保持机架稳固、S1 回中，测试时不操作姿态和偏航摇杆。

下载前断开动力电池。此固件仍会输出真实电机信号；保持拆桨，下载后确认 SA=2000、ARM=0 和三路命令均为120，再安排动力上电。

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/motor_bench_safety_test.cpp -o /tmp/drehmflight-motor-bench-safety-test
/tmp/drehmflight-motor-bench-safety-test
python3 tools/motor_bench/build_image.py
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-capped-bench-build /tmp/drehmflight-capped-bench/dRehmFlight_Teensy_BETA_1.3
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:110000 --build-path /tmp/drehmflight-capped-bench-build /tmp/drehmflight-capped-bench/dRehmFlight_Teensy_BETA_1.3
```

上传端口需按 `arduino-cli board list` 核对。本工具会按特定源代码位置加入防护，若正常 sketch 的相关代码变动而无法定位则直接报错。完成台架排查后必须下载正常 sketch，并确认串口不再显示 `BENCH:1`。

## 单电机位置和转向核对

`--single-motor` 仅支持DShot，在保留限幅和三秒停机的基础上，用直接低油门驱动一个选定电机，忽略姿态混控，并将倾转舵机固定在逻辑中位90。没有选定电机时三路均为0。此配置只加入生成的台架镜像，不修改正常工程。

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/single_motor_test.cpp -o /tmp/drehmflight-single-motor-test
/tmp/drehmflight-single-motor-test
python3 tools/motor_bench/build_image.py --protocol dshot300 --single-motor --output /tmp/drehmflight-single-motor-bench
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-single-motor-bench-build /tmp/drehmflight-single-motor-bench/dRehmFlight_Teensy_BETA_1.3
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:120000 --build-path /tmp/drehmflight-single-motor-bench-build /tmp/drehmflight-single-motor-bench/dRehmFlight_Teensy_BETA_1.3
```

下载前断开动力电池，保持拆桨、SA=2000、油门最低；端口以当前查询为准。发送 `?` 查询，`SINGLE:1 SEL_M:0` 表示尚未选择。USB字符 `A/B/C` 分别选择M2/M3/M4，只在有效链路、SA停机、油门低于1050、未解锁且驱动正常时接受。`X`立即清除选择；遥控链路无效也清除选择。USB选择本身不会解锁或启动电机。

每次试转均先选择、查询SEL_M确认，再低油门以SA=1000解锁，缓慢推油门到约5%–10%。三秒自动停机后将SA回2000、油门最低；未选通道始终发送0，选中通道最大239。不触碰转动中的电机；从上往下看，利用起转/停转瞬间或录像核对位置和CW/CCW。建议位置与转向见[机架配置](../../docs/TRICOPTER_SETUP.md)。

## M3软件反转设置镜像

本次M3实测CW，与安装方案的CCW不同，且三相线已焊接。可额外加 `--m3-direction`，仅与 `--single-motor --protocol dshot300` 组合使用。默认不发送设置，正常工程和其他台架镜像不包含此入口。

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/m3_direction_test.cpp -o /tmp/drehmflight-m3-direction-test
/tmp/drehmflight-m3-direction-test
python3 tools/motor_bench/build_image.py --protocol dshot300 --single-motor --m3-direction --output /tmp/drehmflight-m3-direction-bench
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-m3-direction-bench-build /tmp/drehmflight-m3-direction-bench/dRehmFlight_Teensy_BETA_1.3
```

USB `R` 仅对M3请求DShot21，临时反转其存储方向；每次MCU启动最多完成一次。确认实际变为CCW并停机后，USB `S` 才用于对M3请求DShot12保存当前设置，每次MCU启动最多允许一次保存尝试。两者均要求有效链路、SA停机、油门低于1050、未解锁、驱动正常，且安全状态已持续一秒。请求会清除电机选择，设置期间阻止全部正常油门输出。

每个设置发送10个相同、请求位置1的命令帧，仅M3为21或12，M2/M4均为0；帧间隔至少1 ms，中间循环保持引脚低而不插入零帧，避免打断连续命令计数。最后100 ms全路发送0，超过保存所需等待时间；全部过程采用状态机，不延时阻塞2 kHz主循环。设置期间安全条件消失则中止，并保持零输出。`CFG_CMD/CFG_REP/R_TX/S_TX/CFG_BUSY/CFG_ABORT` 只表明发送情况，不是电调确认；保存后须断开电调动力再上电、重新低油门试转，确认方向保持。

依据：[BLHeli_S原始实现（命令21/12及6次计数）](https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/BLHeli_S.asm)、[Bluejay实现](https://github.com/bird-sanctuary/bluejay/blob/main/src/Modules/DShot.asm)、[Betaflight命令表与保存等待时间](https://betaflight.com/docs/development/API/Dshot)。实际电调固件版本仍未读出，必须用实物转向验证，未验证反转前不保存。

## 动力断开时的控制方向核对

`--protocol dshot300 --control-check` 生成常规混控台架镜像，并只在 `?` 查询中加入 `T_DES`、`R_PID/P_PID/Y_PID`、`M2_S/M3_S/M4_S`。后者是混控后的未限幅尺度，不是实际电机脉冲或转速；SA停机时实际DShot值仍为0。镜像仍保留电机限幅和三秒停机，默认无周期打印，不包含M3设置入口。

```sh
python3 tools/motor_bench/build_image.py --protocol dshot300 --control-check --output /tmp/drehmflight-control-check
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-control-check-build /tmp/drehmflight-control-check/dRehmFlight_Teensy_BETA_1.3
```

初次方向检查必须断开动力电池、SA停机、油门最低，保持摇杆不动；从水平依次将整个机架右侧下倾、左侧下倾、机头下倾、机尾下倾约10°–15°，各静止保持几秒，比较IMU与控制器补偿方向。未限幅数据可避免低油门输出裁剪掩盖混控符号。此检查验证计算方向，不能替代动力上电后的真实闭环响应测试。
