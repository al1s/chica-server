# tools

the harnesses I used to reconstruct and verify ChicaServer against the original
app. most of these expect captured reference traces (kept outside the repo), so
they're here for reference more than for outside use.

| path | what's inside |
| :--- | :------------ |
| `run_exactness_regression.py` | the entry point — runs every oracle below and reports the worst delta |
| `capture/` | pulls reference data off the original app (logcat / Frida hooks, coverage) |
| `oracle/` | diffs the rebuild against the captured traces (gait, quad, walk, animation, calibration) plus the C++ probes |
| `device/` | the board protocol model, the socket/TCP fakes, the emulator↔hardware serial bridge, and the protocol / no-hardware verifiers |

```bash
# run the full exactness regression
python3 tools/run_exactness_regression.py

# bridge the Android emulator to a real board on the host's USB
python3 tools/device/run_servo2040_serial_bridge.py --device /dev/cu.usbmodemXXXX
```
