# Cyclone 开发工具

当前固件为v7 `drehm-cyclone-rotor-v7-stop-20261105`，111项参数回读一致；15 RPM回退增益后完成一轮接近60 RPM的实机记录，用户反馈整体可用但有轻微晃动，实机调试暂时暂停。模型、预设与部署记录见[完整动力学模型](../../docs/CYCLONE_ROTOR_MODEL.md)和[v7停转资格记录](../../docs/CYCLONE_V7_STOP_QUALIFICATION_20261105.md)。应用／设备采用v7 ID，32波形／59状态／111参数／8窗口；转速波形8／9／10／17在Web乘1/6显示RPM，原始RMPLOG仍为°/s。`inspect_recording.py`支持v3／v4／v5／v6／v7历史Profile，不能跨版本解释状态语义。

此前v5方向激光版已完成30°/s（5 RPM）单方向自旋，用户反馈旋转与激光正常，见[v4／v5实测](../../docs/CYCLONE_V4_SPIN_AND_V5_LASER_20261105.md)。该轮32波形／59状态／109参数使用旧v3 ID应用revision2；它的实飞与周期结果不能当作v7硬件验收。v7停转预设 `docs/remote-mcu/cyclone-v7-stop-test-parameters.json` 保留稳定PI 1.5／0.4；60 RPM单项预设 `docs/remote-mcu/cyclone-v7-60rpm-test-parameters.json` 只将参数82设为360°/s，必须在落地、SA停机时使用。60 RPM实机记录已归档，不能将一次记录当作参数辨识完成。

历史水平标定版加入独立找平后的IMU安装角补偿，用户反馈普通模式前飘“现在好很多了”，尚非位置闭环验收。首次低速自旋曾触发超速制动和制动超时，原始诊断保留于[首次自旋记录](../../docs/CYCLONE_SPIN_DIAGNOSTIC_20261105.md)及[水平标定记录](../../docs/CYCLONE_LEVEL_ALIGNMENT_20261105.md)，不代表当前v5状态。动力学与接线以[模型文档](../../docs/CYCLONE_ROTOR_MODEL.md)为准。激光使能后，普通模式解锁时常亮，自旋／制动／归并时仅在广义前方窗口有时限点亮，停机熄灭；状态36包含对齐切入事件，不能当圈数。v5试验预设为 `docs/remote-mcu/cyclone-v5-test-parameters.json`，每次重启后仅停机恢复并回读；分析工具不会下载固件、解锁、电机试转或自动写飞行参数。

2026-11-04下载、Yaw核对、动力地面检查、一次断电恢复和普通模式起飞记录见[硬件会话记录](../../docs/CYCLONE_HARDWARE_SESSION_20261104.md)。普通激光常亮和停机熄灭已有观察；用户确认会话修复版可离地，但沿机体+X前飘。最后一次12.8s电机命令段不等于已知空中时长，整段Yaw最大持向偏差约24.9°；先复核机械水平／机构零位，不据此盲调PID，自旋继续关闭。细节见[前飘复核](../../docs/CYCLONE_NORMAL_HOVER_20261104.md)。原v2软件快照、激光版硬件证据和会话修复版软件／硬件证据分别保留。

## RemoteMCU 应用配置

仅离线验证配置尺寸、UTF-8字节限制、通道／状态／参数ID和引用关系：

```sh
python3 tools/cyclone/profile.py
```

Agent启动后导入应用和设备配置，并从目录回读核验；保持原选择，适用于飞控仍为旧固件时提前准备：

```sh
python3 tools/cyclone/profile.py --install
```

新固件下载、build确认后，再在Web选用新应用，或显式选择：

```sh
python3 tools/cyclone/profile.py --install --select
```

默认Agent为 `http://127.0.0.1:47301`，可通过 `--agent` 指定其他本机端口。工具只允许回环HTTP地址，只调用 `/api/profiles` 和 `/api/profiles/select`；不访问ESP32或MCU。重复执行不会为相同内容增加revision；修改使用Agent当前revision，冲突即停止。Agent生成的revision不一定等于仓库初始revision。

水平标定版应用布局为32通道、58状态、109参数、8窗口。新增参数113／114是机架真实水平时的原始 Roll/Pitch 示值，默认 +0.04°／−0.15°，仅停机可改；不是驾驶姿态配平指令。姿态及三轴角速度波形采用标定后的机体系，AccZ仍为传感器Z轴。若要重新读取原始水平示值，先在停机状态将113／114均设为0。

CSV列 ch01 对应通道ID0；通道31是自旋保护原因位，激光计数保留于状态36，不能套用v1布局。状态47–58和参数100–112用于监督器、停转判据与驾驶坐标交接。旧Yaw分析工具采用不同布局，不能读取本版记录；不能只依据schema1判断数据兼容。后续分析需要同一段的原始RMPLOG、CSV、录制metadata、Profile、前后参数快照和实际离地时间，不用模型预计力矩代替实测力矩。原始RMPLOG按网络接收顺序写入，须按boot隔离、按源序号排序去重并统计缺样，再推进执行器模型；不能在接收顺序中对晚到包直接积分。

本水平标定build的已结束记录可用 `inspect_recording.py RAW --metadata METADATA --boot BOOT [--start SECONDS --end SECONDS]` 作只读JSON核验。metadata接受Agent录制对象或stop响应；工具严格核对build和CRC，拒绝未结束或截断记录，输出源缺样、模式变化、通道统计及窗口末STATE。估计航向变化不是独立物理转角测量；缺样时不插值，不能用本工具的样本均值替代完整动态辨识。

用户已完成[120°往返航向核验](../../docs/CYCLONE_YAW_ANGLE_CHECK_20261105.md)，实际参考角明确为120°，不是90°；方向及角度量级相符，静置残余随时间变化，尚未写入比例或零偏修正。标定停机时飞行／自旋授权均已关闭，后续试飞前需重新检查和恢复所需授权。

## 构建离线模型文档

```sh
npm ci --prefix tools/cyclone --ignore-scripts
node tools/cyclone/build_model.cjs
```

也可把依赖安装在临时目录，将其 `node_modules` 路径作为第一个参数。默认源文件为 `docs/CYCLONE_ROTOR_MODEL.md`，产物为 `docs/reports/cyclone-rotor-model.html`；第二、第三参数可指定Markdown源文件与HTML目标，如本次60 RPM准备报告。构建时使用固定版本的Marked和MathJax，所有公式转成内嵌SVG，生成HTML无需网络、JavaScript或远程字体。

## 软件验证

```sh
python3 tools/run_host_tests.py --sanitize
arduino-cli compile --fqbn teensy:avr:teensy40:usb=serial Versions/dRehmFlight_Teensy_BETA_1.3
```

主机回归包含历史台架接口，`cyclone_spin_model_test.cpp` 对应本版统一物理链路，`cyclone_transition_safety_test.cpp` 检查低速制动、双向180°交接和保护边界。姿态仿真不包含平移、离地／落地接触、真实IMU振动或完整不确定性。硬件周期、实际激光相位和无吊绳飞行表现必须另外验证。
