# Cyclone 开发工具

本目录用于 `drehm-cyclone-rotor-v1-20261103`。动力学与接线以[模型文档](../../docs/CYCLONE_ROTOR_MODEL.md)为准。工具不会下载固件、解锁、电机试转或自动写飞行参数。

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

应用布局为32通道、46状态、94参数、8窗口。CSV列 ch01 对应通道ID0。旧Yaw分析工具采用不同布局，不能读取本版记录；不能只依据schema1判断数据兼容。后续分析需要同一段的原始RMPLOG、CSV、录制metadata、Profile、前后参数快照和实际离地时间，不用模型预计力矩代替实测力矩。

## 构建离线模型文档

```sh
npm ci --prefix tools/cyclone --ignore-scripts
node tools/cyclone/build_model.cjs
```

也可把依赖安装在临时目录，将其 `node_modules` 路径作为构建脚本的第一个参数。内容只编辑 `docs/CYCLONE_ROTOR_MODEL.md`；构建产物为 `docs/reports/cyclone-rotor-model.html`。构建时使用固定版本的Marked和MathJax，所有公式转成内嵌SVG，生成HTML无需网络、JavaScript或远程字体。

## 软件验证

```sh
python3 tools/run_host_tests.py --sanitize
arduino-cli compile --fqbn teensy:avr:teensy40:usb=serial Versions/dRehmFlight_Teensy_BETA_1.3
```

主机回归包含历史台架接口，只有 `cyclone_spin_model_test.cpp` 的统一链路用例对应本版自旋控制。其Newton–Euler姿态仿真不包含平移、离地／落地接触、真实IMU振动或完整不确定性。硬件周期、实际激光相位和无吊绳飞行表现必须另外验证。
