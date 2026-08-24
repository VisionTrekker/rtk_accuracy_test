# rtk_accuracy_test

BX100 双 GNSS 基准站/移动站的 ROS 2 Humble RTK 测试驱动、低权限串口录制器与离线分析工具。

---

## 📋 文档入口

| 文档 | 内容 |
|---|---|
| [项目背景与原始需求](../QY_RTK.md) | 接收机消息格式和测试需求 |
| [外场测试方案](../RTK_TEST_PLAN.md) | 静态点、局部测试网、公里级闭环与现场记录表 |
| [未知配置冒烟测试](../docs/20260821_unknown_config_smoke_test.md) | 兼容性验证，不属于正式精度结果 |
| [5 Hz 接收机冒烟测试](../docs/20260824_5hz_receiver_smoke_test.md) | 带宽与数据完整性验证，不属于正式精度结果 |

## 🗂️ 工具与运行环境

`src/bx100_test_recorder.cpp` 是 ROS 2 节点，面向开发笔记本或有完整权限的计算机。
`scripts/serial_capture.py` 是纯 Python 标准库录制器，面向小车低权限 RK3588：不需要
ROS、`pyserial`、编译器或 `sudo`。

| 工具 | 运行位置 | 用途 |
|---|---|---|
| `scripts/serial_capture.py` | 低权限 RK3588 | 保存原始串口字节流和逐行主机时间索引 |
| `bx100_test_recorder` | 开发机 | 发布 ROS 2 话题并保存原始串口行 |
| `scripts/rtk_offline_analysis.py` | 开发机 | 清点消息、校验帧、导出 GGA/ENU 初步结果 |
| `scripts/rtk_amap_view.py` | 开发机 | 将校验通过的 GGA 原始轨迹导出为高德地图 HTML |

## 🔧 RK3588 低权限采集

从工作区根目录复制录制器到 RK3588：

```bash
scp rtk_accuracy_test/scripts/serial_capture.py <user>@<rk3588_ip>:/home/<user>/
```

进入 RK3588 后确认串口设备（例如 `/dev/ttyUSB0`），录制 5 分钟：

```bash
mkdir -p ~/rtk_data/session_001
python3 ~/serial_capture.py \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --duration 300 \
  --raw-output ./data/rover_20260824_01.raw \
  --index-output ./data/rover_20260824_01.tsv
```

持续录制时省略 `--duration`，按 `Ctrl-C` 停止：

```bash
python3 ~/serial_capture.py \
  --port /dev/ttyUSB0 \
  --raw-output ./data/rover_20260824_01.raw \
  --index-output ./data/rover_20260824_01.tsv
```

脚本生成 `.raw`（完整原始字节流）与 `.tsv`（每条完整串口行及 RK3588 接收时间）。
接收机 UTC 仍保留在 `GGA`、`ZDA`、`TIMEA` 等原始消息中。录制结束后取回数据：

```bash
scp -r <user>@<rk3588_ip>:/home/<user>/rtk_data ./data/
```

建议在 RK3588 使用 `tmux`，避免 SSH 中断停止录制：

```bash
tmux new -s rtk_capture
# 在 tmux 中执行录制命令；Ctrl-B 后按 D 可退出但保持运行
tmux attach -t rtk_capture
```

## ⚙️ ROS 2 Humble 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_accuracy_test --symlink-install
source install/setup.bash
```

## 📡 ROS 2 移动站录制

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

`GGA` talker IDs are not restricted: for example, `$GNGGA` is parsed as GGA.
When `BESTPOSA` is absent, `/rtk/status` maps the NMEA GGA quality field to a
conservative class (`2` is `DGPS`, `4` is `RTK_FIXED`, and `5` is
`RTK_FLOAT`). `HDT` sentences are retained in `/rtk/status` but do not contain
solution quality or standard deviation, so `/heading` is not published from
HDT by default. Set `publish_unqualified_hdt:=true` only after its source and
validity have been verified.

## 📊 离线初步分析

```bash
python3 scripts/rtk_offline_analysis.py data/session_001/rover_*.raw \
  --origin LAT LON HEIGHT --csv data/session_001/analysis/gga_enu.csv \
  --summary data/session_001/analysis/summary.txt
```

The current analyzer computes parsed GGA/ENU rows and quality counts. Static-
point windows, test-net distances, event alignment, and loop closure should be
extended after the first real receiver logs confirm the exact BX100 `BESTPOSA`
field order and state-enumeration values.

### 高德地图轨迹回放

地图导出器直接读取原始录制，不依赖中间 CSV；它只绘制 NMEA 校验和正确、经纬度
完整的 GGA 样本。轨迹根据 GGA 质量字段分为 RTK Fixed、RTK Float、差分、单点等
不同颜色的连续段，状态变化处会断线。因此它适合回放和诊断，不能作为绝对精度结论。

```bash
python3 scripts/rtk_amap_view.py \
  data/rover_20260824_01.raw \
  data/rover_20260824_01_amap.html
```

在浏览器打开 HTML 后，在左上角输入高德 Web JS API Key 并点击“加载”。轨迹会由
WGS-84 转为 GCJ-02 后叠加到高德底图，起点和终点可点击查看原始 WGS-84 坐标、高程、
GGA UTC、解状态以及（若原始行包含）主机接收时间。未带时间戳的纯 `.raw` 字节流不会
伪造主机时间。

可选地传入 `--amap-key` 和 `--security-js-code` 预填凭据，但它们会以明文写进 HTML；
不要将这种 HTML 分享或提交到仓库。默认的浏览器输入仅保存到该浏览器的
`localStorage`。

For the one-notebook base-station checks, disconnect the rover, connect the
base station, and use `config/base.yaml` (or `ros2 launch
rtk_accuracy_test base.launch.py`). Record `base_before.raw` and
`base_after.raw` in separate session directories; do not connect both
receivers to the notebook at the same time.

## 🔐 数据与提交约定

原始串口数据、rosbag、构建目录、日志、IDE 配置、压缩包和密钥文件均被工作区
`.gitignore` 排除。提交前检查：

```bash
git status --short
git diff --cached --stat
```
