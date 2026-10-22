# Third Party Notices

## CRSFforArduino

This firmware uses CRSFforArduino by Cassandra "ZZ Cat" Robinson, version
2025.12.11, to receive ExpressLRS CRSF channel data. The library is distributed
under the GNU Affero General Public License v3.0 (AGPL-3.0).

- Source: https://github.com/ZZ-Cat/CRSFforArduino
- License text: https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md
- Arduino Library Manager package: `CRSFforArduino@2025.12.11`

Install the package through Arduino IDE Library Manager or with
`arduino-cli lib install CRSFforArduino@2025.12.11` before compiling this sketch.

For this 2 kHz tricopter build, run `python3 tools/configure_crsf_realtime.py`
from the repository root after installing the library. It sets
`CRSF_TELEMETRY_ENABLED` to `0` in the installed library configuration. This
disables unused outgoing CRSF telemetry and avoids waiting for UART transmission
inside the receiver library's `flushRemainingFrames()`. Incoming RC channels and
link statistics remain enabled. The sketch checks this setting at compile time.
