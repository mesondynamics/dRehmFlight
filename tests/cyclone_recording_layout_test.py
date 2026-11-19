"""Guard the incompatible v12 and v13 meanings of four waveform slots."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "cyclone"))
from inspect_recording import HOVER_SLOPE_BUILD, read_recording, summarize


V12 = "drehm-cyclone-rotor-v12-dshot-resolution-20261119"


def sample_boot(build):
    hello = json.dumps({"build_id": build}).encode()
    values = [0.] * 32
    values[3] = 123.
    values[11:15] = [1.125, 700.5, .25, -.25]
    values[22] = .875
    sample = struct.pack("<IIHBB32f", 1, 4, 10000, 1, 32, *values)
    log = bytearray(b"RMPLOG1\n")
    for seq, (kind, payload) in enumerate(((1, hello), (16, sample))):
        frame = struct.pack("<4sBBHHHIIQ", b"RMP1", 1, kind, 0, len(payload), 0, 10, seq, 50000)
        frame += payload
        frame += struct.pack("<I", zlib.crc32(frame))
        log.extend(struct.pack("<QI", 80000, len(frame)) + frame)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "synthetic.rmplog"
        path.write_bytes(log)
        sessions, _, _ = read_recording(path)
    return sessions[10]


class CycloneRecordingLayoutTest(unittest.TestCase):
    def test_channel_names_follow_the_firmware(self):
        for build, expected in (
            (V12, ["requested_torque_Nm", "predicted_torque_Nm",
                   "feedforward_torque_Nm", "integral_rad_s2"]),
            (HOVER_SLOPE_BUILD, ["world_up_specific_force_filtered_g", "mean_wire_dshot_filtered_code",
                                 "sensor_acc_x_g", "sensor_acc_y_g"]),
        ):
            with self.subTest(build=build):
                channels = summarize(sample_boot(build), 0, 1, build)["channels"]
                self.assertEqual(list(channels)[11:15], expected)
                self.assertEqual([channels[name]["mean"] for name in expected], [1.125, 700.5, .25, -.25])
                self.assertEqual(channels["phase_deg"]["first"], 123.)
                self.assertEqual(channels["sensor_acc_z_g"]["mean"], .875)

    def test_mismatched_recording_metadata_rejected(self):
        with self.assertRaisesRegex(ValueError, "firmware differs"):
            summarize(sample_boot(V12), 0, 1, HOVER_SLOPE_BUILD)


if __name__ == "__main__":
    unittest.main()
