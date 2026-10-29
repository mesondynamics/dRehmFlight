# DShot300 / DShot600 电调协议台架排查

本文件记录用于验证 RuiBet 65A 协议兼容性的拆桨台架固件；该台架镜像不能起飞。当前板上已切换正式DShot300配置，最新验收进度见[三旋翼配置](TRICOPTER_SETUP.md)。电调出厂固件版本未知；用户已用万用表确认动力供电、通道连接和共地，Teensy 三路 GPIO 的 OneShot 回读已通过，但电机未接受此前的试转命令。

[Betaflight 协议说明](https://betaflight.com/docs/development/API/Dshot)指出 BLHeli_S 支持 DShot，Bluejay 支持 DShot 而不支持模拟脉宽协议；[JHEMCU 产品页](https://www.jhemcu.com/e_productshow/?82-JHEMCU-RUIBET-6065A-3-6S-DSHOT600-BLHELI_S-4IN1-ESC-82.html=)标注 RuiBet 60/65A 的 DShot600 支持。因此先测试共同支持的数字协议，不修改电调内部固件或持久设置。

## 实现与限幅

- Teensy 4.0 数字 1/2/3 对应 M2/M3/M4，沿用原连接。
- 三路并行发送标准单向 DShot300 / DShot600，16位数据帧。运行帧不请求遥测；台架允许通过 USB 单独触发 BEEP1 声响检查，不发送方向、保存设置等持久命令。
- 使用600 MHz CPU的DWT计数器定时；仅帧发送期间屏蔽中断，并恢复进入前的PRIMASK。GPIO写入时序异常会锁定驱动，保持引脚低电平。
- 现有主循环仍为2 kHz，舵机PWM定时器不变。临时台架固件保留三秒停机和SA复位条件；最低油门不发运行值。
- 原内部125–137缩放量转换为最大DShot 239；未解锁、失联、SA停机及超时均发送0。`M2_US/M3_US/M4_US`在此镜像中是原算法内部缩放量，不是线上脉宽；`D2/D3/D4`才是发送的DShot值。
- `TX_NS_MAX`是软件计量的帧发送窗口；`IRQ_NS_MAX`是屏蔽中断窗口；`EDGE_NS_MAX`是GPIO写入相对软件目标时刻的最大延迟，不能替代示波器对线路模拟质量的测量；`DFAULT=0`表示未触发计时故障锁定。

## 生成与下载

先拆桨、断开动力电池，保持油门最低、SA=2000、S1回中；下载后确认停机诊断，再安排电调上电。

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/dshot_protocol_test.cpp -o /tmp/drehmflight-dshot-protocol-test
/tmp/drehmflight-dshot-protocol-test
python3 tools/motor_bench/build_image.py --protocol dshot600 --output /tmp/drehmflight-dshot-bench
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-dshot-bench-build /tmp/drehmflight-dshot-bench/dRehmFlight_Teensy_BETA_1.3
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:110000 --build-path /tmp/drehmflight-dshot-bench-build /tmp/drehmflight-dshot-bench/dRehmFlight_Teensy_BETA_1.3
```

板卡端口须按实际查询结果替换。正常工程现已默认选择DShot300，共用经过台架验证的驱动、GPIO故障锁定及Serial5 FIFO配置；OneShot125作为可选旧分支保留。生成器在仓库外加入限幅、三秒停机和USB蜂鸣功能。正式工程的完整飞行验收尚未完成，当前板上已切换取消限幅与三秒停机的正式镜像，带动力失联与首次悬停验收待完成。

## 按需诊断和蜂鸣检查

默认关闭周期打印；串口发送 `?` 才返回一行状态，每次循环最多消费一个输入字节，诊断回复最多20 Hz，不等待串口读者。正常飞行源代码也采用此查询方式。

本台架镜像中，串口发送单个字符 `2`、`3`、`4`，分别请求 M2、M3、M4 发出一次 BEEP1。要求有效遥控链路、SA停机、油门低于1050、未解锁，且此安全状态已持续至少一秒。若条件不满足直接丢弃。命令帧仅目标通道为1，并置请求位；其他通道为0。随后500 ms全通道保持0，不阻塞循环；此间不能再请求蜂鸣。`BEEP_M`与`BEEP_N`记录实际发出的目标和次数，不能证明电调已经收到。

依据 [Betaflight DShot 命令表](https://betaflight.com/docs/development/API/Dshot)，BEEP1是声响命令，需要至少260 ms间隔；请求位处理参考 [Betaflight 命令发送实现](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_command.c)。此测试用于辨别协议接收与低油门起转问题；听到上电提示音本身不足以确认协议。

主机检查：

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/dshot_beep_test.cpp -o /tmp/drehmflight-dshot-beep-test
/tmp/drehmflight-dshot-beep-test
```

2026-10-15 更新后，无查询时连续一秒收到0字节；15次查询均得到回复。循环周期及最大周期均500 µs，最大工作时间243 µs，OVERRUN=0，DFAULT=0，三路运行值均0。

此前DShot600试转记录：用户报告三个电机均未转，采集的实际命令上限239，原始CH3最高1638，未触发计时故障、循环超时或遥控失联。因此起转及协议接收尚需声响测试继续确认。

另一次断开动力电池的摇杆复现中，CH1动作期间CH3从1000升至1822，三秒后TIMEOUT由0变1；此后CH4仍覆盖1000–2000，舵机指令固定90。已确认这是台架超时撤销解锁造成的回中，用户要求保留现有保护、继续排查电调。正常飞行源代码没有此三秒定时。

## 下载后验证

断电情况下8秒采集161条诊断：`PROTO=600 BENCH=1 DCAP=239 DFAULT=0`，三路DShot值均为0，遥控链路有效、SA=2000、ARM=0。帧发送窗口最大26,711 ns，中断屏蔽窗口最大26,718 ns，GPIO写入最大延迟38 ns；主循环与最大周期均500 µs，最大工作时间243 µs，超时计数0。数据帧的官方示例、全量油门校验、特殊命令排除和三路位序主机检查通过。此条记录尚不包含实际电机起转结果。

## DShot300 对比（2026-10-15）

DShot600的两次M2短鸣未得到相应声响，不能把之后偶发单鸣视为确认响应。改用DShot300后，按M2→M3→M4、间隔两秒分别发一次BEEP1，用户确认三次短鸣均对应听到；三个通道的协议接收得到实际验证。不能仅凭本次对比认定该型号不支持DShot600，当前板卡固件/线路条件下600失败的具体原因仍未定位。

生成器新增 `--protocol dshot300`。保持2 kHz控制循环及原有运行限幅239、三秒停机。DShot300一帧约53.3 µs；为避免420k CRSF接收在屏蔽中断期间挤满四字节FIFO，仅台架镜像将Teensy4.0 Serial5/LPUART8的RXWATER从2降至1，较早触发RX中断，不修改已安装的Teensy核心。

DShot驱动每64帧在三路GPIO高段、0/1区分段、低段读取实际引脚PSR电平。`PAD_N`是抽查帧数，`PAD_ERR`是错误通道位掩码（bit0/1/2对应M2/M3/M4）；异常会锁停驱动。此回读不能测量ESC端电压或替代示波器。`RX_OR`记录UART接收溢出标志。

断电基线28次查询：PAD_N从1196增至1377，PAD_ERR=0、RX_OR=0、DFAULT=0；循环及最大周期500 µs，最大工作时间276 µs，OVERRUN=0。三路蜂鸣前后，同样无引脚回读错误或循环超时。

```sh
python3 tools/motor_bench/build_image.py --protocol dshot300 --output /tmp/drehmflight-dshot300-bench
arduino-cli compile --fqbn teensy:avr:teensy40 --build-path /tmp/drehmflight-dshot300-bench-build /tmp/drehmflight-dshot300-bench/dRehmFlight_Teensy_BETA_1.3
arduino-cli upload --fqbn teensy:avr:teensy40 --port usb:110000 --build-path /tmp/drehmflight-dshot300-bench-build /tmp/drehmflight-dshot300-bench/dRehmFlight_Teensy_BETA_1.3
```

### 试转成功与正常工程整理

DShot300低油门测试中，用户确认三个电机均起转。串口377次查询，运行样本9条，原始CH3最高1098，三路最大运行值M2=223、M3=239、M4=239。TIMEOUT锁定生效；最终SA=2000、CH3=1000、ARM=0，三路DShot值均0。最大循环周期500 µs、最大工作时间278 µs、OVERRUN=0、PAD_ERR=0、RX_OR=0、DFAULT=0。

正常工程已设置 `DREHM_ESC_DSHOT_RATE=300000`，使用相同驱动与FIFO配置，默认无周期打印；发送 `?` 返回PROTO和D2/D3/D4。正常工程不包含台架三秒定时、额外电机限幅或USB蜂鸣入口。本轮仅编译整理后的正常工程，板上仍为上述实际验证的DShot300台架镜像。
