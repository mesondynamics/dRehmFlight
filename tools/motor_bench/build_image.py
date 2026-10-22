#!/usr/bin/env python3
"""Generate an isolated capped motor bench sketch; never edits the flight sketch."""
from pathlib import Path
import argparse
import shutil
import re


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"Flight source changed; expected one occurrence: {old!r}")
    return text.replace(old, new, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/drehmflight-capped-bench"))
    parser.add_argument("--protocol", choices=["oneshot125", "dshot600", "dshot300"], default="oneshot125")
    parser.add_argument("--single-motor", action="store_true",
                        help="USB A/B/C selects M2/M3/M4 while disarmed; throttle drives only that motor")
    parser.add_argument("--m3-direction", action="store_true",
                        help="Guarded M3-only temporary reverse (R) and subsequent save (S)")
    parser.add_argument("--control-check", action="store_true",
                        help="Mixed DShot300 bench with on-demand PID and unclamped mixer diagnostics")
    args = parser.parse_args()
    if args.single_motor and not args.protocol.startswith("dshot"):
        parser.error("--single-motor requires DShot")
    if args.m3_direction and (not args.single_motor or args.protocol != "dshot300"):
        parser.error("--m3-direction requires --single-motor --protocol dshot300")
    if args.control_check and (args.single_motor or args.m3_direction or args.protocol != "dshot300"):
        parser.error("--control-check requires mixed --protocol dshot300")
    root = Path(__file__).resolve().parents[2]
    source = root / "Versions/dRehmFlight_Teensy_BETA_1.3"
    destination = args.output.resolve() / source.name
    if destination == source.resolve() or root.resolve() in destination.parents:
        raise RuntimeError("Bench output must be outside the repository")
    shutil.copytree(source, destination, dirs_exist_ok=True)
    bitrate = {"oneshot125": 0, "dshot300": 300000, "dshot600": 600000}[args.protocol]
    config = destination / "TricopterConfig.h"
    config.write_text(replace_once(config.read_text(),
                                  "#define DREHM_ESC_DSHOT_RATE 300000",
                                  f"#define DREHM_ESC_DSHOT_RATE {bitrate}"))
    shutil.copy2(Path(__file__).with_name("MotorBenchSafety.h"), destination)
    sketch = destination / (source.name + ".ino")
    text = sketch.read_text()
    text = replace_once(text, '#include "TricopterConfig.h"',
                        '#include "TricopterConfig.h"\n#include "MotorBenchSafety.h"')
    text = replace_once(text, "bool armedFly = false;",
                        "motor_bench::Safety benchSafety;\nbool armedFly = false;")
    text = replace_once(text, "void armedStatus() {\n",
                        "void armedStatus() {\n"
                        "  if (benchSafety.timedOut()) { armedFly = false; return; }\n")
    marker = "  //Scaled to 0-180 for servo library"
    text = replace_once(text, marker,
                        "  m2_command_PWM = benchSafety.capPulse(m2_command_PWM);\n"
                        "  m3_command_PWM = benchSafety.capPulse(m3_command_PWM);\n"
                        "  m4_command_PWM = benchSafety.capPulse(m4_command_PWM);\n\n" + marker)
    text = replace_once(text, "void throttleCut() {\n",
                        "void throttleCut() {\n"
                        "  if (!benchSafety.allow(armedFly, channel_5_pwm, channel_1_pwm, current_time))\n"
                        "    armedFly = false;\n")
    text = replace_once(text, '"ARM:%u FULLTEST:%u',
                        '"ARM:%u BENCH:1 CAP_US:137 TIMEOUT:%u FULLTEST:%u')
    text = replace_once(text,
                        "(unsigned)armedFly, (unsigned)tricopter::fullRangeBenchTest,",
                        "(unsigned)armedFly, (unsigned)benchSafety.timedOut(), "
                        "(unsigned)tricopter::fullRangeBenchTest,")
    if args.protocol.startswith("dshot"):
        shutil.copy2(Path(__file__).with_name("DShotBeep.h"), destination)
        text = replace_once(text, '#include "MotorBenchSafety.h"',
                            '#include "MotorBenchSafety.h"\n#include "DShotBeep.h"')
        text = replace_once(text, "motor_bench::Safety benchSafety;",
                            "motor_bench::Safety benchSafety;\n"
                            "motor_bench::DShotBeep benchBeep;")
        text = replace_once(text, "if (benchSafety.timedOut())",
                            "if (benchSafety.timedOut() || !dshotOutput.ready())")
        pattern = r"^void commandMotors\(\) \{.*?^\}"
        adapter = '''bool benchBeepSafe() {
  bool receiverReady = true;
#if defined USE_CRSF_RX
  receiverReady = crsfChannelsReady && crsfLinkUp && !crsfFailsafe;
#endif
  return receiverReady && !armedFly && channel_5_pwm >= 1500 &&
         channel_1_pwm < 1050 && dshotOutput.ready();
}

void handleBenchUsb() {
  if (Serial.available() <= 0) return;
  const int command = Serial.read(); // At most one byte per control iteration.
  if (command == '?') printGyroData();
  else if (command >= '2' && command <= '4')
    benchBeep.request(command - '0', benchBeepSafe(), current_time);
}

void commandMotors() {
  uint16_t beepValues[3];
  uint8_t telemetryMask;
  if (benchBeep.frame(benchBeepSafe(), micros(), beepValues, telemetryMask)) {
    dshotOutput.send(beepValues[0], beepValues[1], beepValues[2], telemetryMask);
    return;
  }
  // Bench protocol adapter: keep motors stopped at idle.
  const bool enabled = armedFly && channel_1_pwm > 1040 && dshotOutput.ready();
  dshotOutput.send(dshot::fromPulseScale(m2_command_PWM, enabled),
                   dshot::fromPulseScale(m3_command_PWM, enabled),
                   dshot::fromPulseScale(m4_command_PWM, enabled));
}'''
        text, count = re.subn(pattern, lambda _: adapter, text, count=1, flags=re.M | re.S)
        if count != 1:
            raise RuntimeError("Cannot locate commandMotors")
        text = replace_once(text,
                            "  if (Serial.available() > 0 && Serial.read() == '?') printGyroData();",
                            "  handleBenchUsb();")
        text = replace_once(text, "Serial.availableForWrite() < 512", "Serial.availableForWrite() < 640")
        text = replace_once(text, "char line[512];", "char line[640];")
        text = replace_once(text, "BENCH:1 CAP_US:137", "BENCH:1 CAP_US:137 DCAP:239")
        text = replace_once(text, '"Roll:%.2f',
                            '"BEEP_M:%u BEEP_N:%lu PAD_N:%lu PAD_ERR:%u RX_OR:%u TX_NS_MAX:%lu IRQ_NS_MAX:%lu EDGE_NS_MAX:%lu DFAULT:%u Roll:%.2f')
        text = replace_once(text, "      (double)roll_IMU,",
                            "      benchBeep.lastMotor(), (unsigned long)benchBeep.count(),\n"
                            "      (unsigned long)dshotOutput.padFrames(), dshotOutput.padErrorMask(), "
                            "(unsigned)!!(IMXRT_LPUART8.STAT & LPUART_STAT_OR),\n"
                            "      (unsigned long)dshotOutput.maxFrameNs(), "
                            "(unsigned long)dshotOutput.maxBlockedNs(), "
                            "(unsigned long)dshotOutput.maxLateNs(), (unsigned)dshotOutput.fault(),\n"
                            "      (double)roll_IMU,")
        if args.control_check:
            # The normal image now exposes roll/pitch PID snapshots too.
            # Keep only the richer control-check fields in this profile.
            text = replace_once(text, '      "R_PID:%.4f P_PID:%.4f "\n', '')
            text = replace_once(text, '      (double)roll_PID, (double)pitch_PID,\n', '')
            text = replace_once(text, 'DFAULT:%u Roll:%.2f',
                                'DFAULT:%u CONTROL_CHECK:1 T_DES:%.3f R_PID:%.4f P_PID:%.4f Y_PID:%.4f '
                                'M2_S:%.4f M3_S:%.4f M4_S:%.4f Roll:%.2f')
            text = replace_once(text, "      (double)roll_IMU,",
                                "      (double)thro_des, (double)roll_PID, (double)pitch_PID, (double)yaw_PID,\n"
                                "      (double)m2_command_scaled, (double)m3_command_scaled, (double)m4_command_scaled,\n"
                                "      (double)roll_IMU,")
            text = replace_once(text, "Serial.availableForWrite() < 640", "Serial.availableForWrite() < 768")
            text = replace_once(text, "char line[640];", "char line[768];")
        if args.single_motor:
            shutil.copy2(Path(__file__).with_name("SingleMotor.h"), destination)
            text = replace_once(text, '#include "DShotBeep.h"',
                                '#include "DShotBeep.h"\n#include "SingleMotor.h"')
            text = replace_once(text, "motor_bench::DShotBeep benchBeep;",
                                "motor_bench::DShotBeep benchBeep;\n"
                                "motor_bench::SingleMotor singleMotor;")
            text = replace_once(text, "  else if (command >= '2' && command <= '4')",
                                "  else if (command >= 'A' && command <= 'C')\n"
                                "    singleMotor.select(command - 'A' + 2, benchBeepSafe());\n"
                                "  else if (command == 'X') singleMotor.select(0, true);\n"
                                "  else if (command >= '2' && command <= '4')")
            text = replace_once(text, "\n  scaleCommands(); //Internal motor scale",
                                "\n  // Single-motor identification: hold the tilt linkage centered.\n"
                                "  s1_command_scaled = tricopter::servoCenterDeg / 180.0f;\n"
                                "  scaleCommands(); //Internal motor scale")
            normal_send = """  dshotOutput.send(dshot::fromPulseScale(m2_command_PWM, enabled),
                   dshot::fromPulseScale(m3_command_PWM, enabled),
                   dshot::fromPulseScale(m4_command_PWM, enabled));"""
            single_send = """#if defined USE_CRSF_RX
  if (!crsfChannelsReady || !crsfLinkUp || crsfFailsafe)
    singleMotor.select(0, true);
#endif
  // Direct low throttle, without attitude mixing, only on the selected motor.
  const unsigned throttle = channel_1_pwm < 1000 ? 1000 :
                            (channel_1_pwm > 2000 ? 2000 : channel_1_pwm);
  const int pulse = benchSafety.capPulse(125 + (throttle - 1000) * 125 / 1000);
  uint16_t values[3];
  singleMotor.route(dshot::fromPulseScale(pulse, enabled), enabled, values);
  dshotOutput.send(values[0], values[1], values[2]);"""
            text = replace_once(text, normal_send, single_send)
            text = replace_once(text, "BENCH:1 CAP_US:137", "BENCH:1 SINGLE:1 SEL_M:%u CAP_US:137")
            text = replace_once(text, "(unsigned)armedFly, (unsigned)benchSafety.timedOut(),",
                                "(unsigned)armedFly, singleMotor.selected(), (unsigned)benchSafety.timedOut(),")
            if args.m3_direction:
                shutil.copy2(Path(__file__).with_name("M3Direction.h"), destination)
                text = replace_once(text, '#include "SingleMotor.h"',
                                    '#include "SingleMotor.h"\n#include "M3Direction.h"')
                text = replace_once(text, "motor_bench::SingleMotor singleMotor;",
                                    "motor_bench::SingleMotor singleMotor;\n"
                                    "motor_bench::M3Direction m3Direction;")
                text = replace_once(text, "  else if (command >= 'A' && command <= 'C')",
                                    "  else if (command == 'R') {\n"
                                    "    if (m3Direction.reverse(benchBeepSafe(), current_time)) singleMotor.select(0, true);\n"
                                    "  } else if (command == 'S') {\n"
                                    "    if (m3Direction.save(benchBeepSafe(), current_time)) singleMotor.select(0, true);\n"
                                    "  } else if (command >= 'A' && command <= 'C')")
                text = replace_once(text, "  uint16_t beepValues[3];",
                                    "  uint16_t configValues[3];\n  uint8_t configMask;\n  bool sendConfigFrame;\n"
                                    "  if (m3Direction.frame(benchBeepSafe(), micros(), configValues, configMask, sendConfigFrame)) {\n"
                                    "    singleMotor.select(0, true);\n"
                                    "    if (sendConfigFrame) dshotOutput.send(configValues[0], configValues[1], configValues[2], configMask);\n"
                                    "    return;\n  }\n  uint16_t beepValues[3];")
                text = replace_once(text, '"BEEP_M:%u',
                                    '"M3CFG:1 CFG_CMD:%u CFG_REP:%u R_TX:%u S_TX:%u CFG_BUSY:%u CFG_ABORT:%lu BEEP_M:%u')
                text = replace_once(text, "      benchBeep.lastMotor(),",
                                    "      m3Direction.lastCommand(), m3Direction.repeats(), "
                                    "(unsigned)m3Direction.reversedTransmitted(), (unsigned)m3Direction.saveTransmitted(), "
                                    "(unsigned)m3Direction.busy(), (unsigned long)m3Direction.aborts(),\n"
                                    "      benchBeep.lastMotor(),")
                text = replace_once(text, "Serial.availableForWrite() < 640", "Serial.availableForWrite() < 768")
                text = replace_once(text, "char line[640];", "char line[768];")
    sketch.write_text(text)
    print(destination)


if __name__ == "__main__":
    main()
