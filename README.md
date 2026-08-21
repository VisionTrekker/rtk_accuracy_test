# rtk_accuracy_test

ROS 2 Humble package for the BX100 field-test plan in
[`../RTK_TEST_PLAN.md`](../RTK_TEST_PLAN.md). This package is intended for a
development laptop or a fully provisioned computer; it is **not required on
the low-privilege RK3588 board**. Use the root-level
[`../serial_capture.py`](../serial_capture.py) and
[`../serial_capture.sh`](../serial_capture.sh) for minimal board-side
recording.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_accuracy_test --symlink-install
source install/setup.bash
```

## Record the rover

Set the receiver to emit the configured logs on `COM1`, connect only the rover
to the notebook, and run:

```bash
ros2 run rtk_accuracy_test bx100_test_recorder --ros-args \
  -p port:=/dev/ttyACM0 -p baud_rate:=115200 -p log_directory:=./data/session_001
```

The node writes a role-prefixed timestamped `rover_*.raw` file (Unix receive time in ns,
tab, original line) and publishes `/fix`, `/heading`, `/rtk/status`, and
`/rtk/raw`. `/rtk/status` is diagnostic and preserves the receiver state;
unknown receiver position-type strings are not treated as fixed.

## Offline first pass

```bash
python3 scripts/rtk_offline_analysis.py data/session_001/rover_*.raw \
  --origin LAT LON HEIGHT --csv data/session_001/analysis/gga_enu.csv \
  --summary data/session_001/analysis/summary.txt
```

The current analyzer computes parsed GGA/ENU rows and quality counts. Static-
point windows, test-net distances, event alignment, and loop closure should be
extended after the first real receiver logs confirm the exact BX100 `BESTPOSA`
field order and state-enumeration values.

For the one-notebook base-station checks, disconnect the rover, connect the
base station, and use `config/base.yaml` (or `ros2 launch
rtk_accuracy_test base.launch.py`). Record `base_before.raw` and
`base_after.raw` in separate session directories; do not connect both
receivers to the notebook at the same time.
