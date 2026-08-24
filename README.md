# rtk_accuracy_test

面向 BX100Y/TX100Y 双天线 RTK 外场精度测试的 ROS 2 Humble 串口驱动、低权限原始数据录制和离线诊断工具。仓库服务于“先完整记录、再可追溯分析”的测试流程：正式结论评价相对精度、重复性、局部几何一致性、闭环一致性和解状态可用率。

基准站没有可信控制点时采用静止自主优化；其坐标只用作同一会话内的 ENU 原点，**不能作为绝对真值**。因此本仓库不应输出或宣传绝对全球坐标精度。当前阶段也不评价杆臂补偿、LIO 融合或车体航向安装零偏。

## 当前状态

| 项目 | 状态 |
|---|---|
| ROS 2 串口节点 | 已实现 `bx100_test_recorder`：发布 `/fix`、`/heading`、`/rtk/status`、`/rtk/raw`，并保存原始串口行。 |
| 定位与航向解析 | 已支持通用 GGA、GST、`BESTPOSA`、`UNIHEADINGA`，并校验厂家 ASCII CRC-32。GGA 质量字段只作保守回退；未知厂家状态绝不提升为 Fixed。 |
| 损坏帧保护 | 已按嵌入的 `$`/`#` 帧头重同步，防止损坏长诊断帧吞掉后续有效位置帧。HDT 仅作无质量航向诊断，默认不发布 `/heading`。 |
| 低权限录制 | 已提供纯 Python `serial_capture.py`，无需 ROS、`pyserial`、编译器或 `sudo`，生成完整字节流 `.raw` 和逐行主机时间索引 `.tsv`。 |
| 离线诊断 | 已提供消息清点、NMEA/厂家 CRC 校验、GGA 解状态统计、基础 ENU CSV 和高德轨迹 HTML 导出。 |
| 自动化验证 | C++ 节点构建、Python 语法检查、录制器安装路径和帮助命令均已验证；当前尚未定义 CTest 自动化测试。 |

### 已验证数据的边界

以下数据只验证代码路径、带宽或录制完整性，**都不是 BX100Y/TX100Y 正式外场精度证据**：

| 数据 | 已确认结果 | 不能得出的结论 |
|---|---|---|
| `./data/rover_20260824_01.raw` | 另一型号接收机的 117.4 s、约 5 Hz 记录；588 条 GGA 质量 `4`，588 条 `BESTPOSA` 为 `NARROW_INT`。115200 下有效负载约 3,555 B/s（30.9%）。 | 接收机内部标准差不是实测外场精度；其结果不能外推到目标机。 |

记录中发现 `SATSINFOA` 24 帧中 20 帧 CRC 无效，且 6 个物理行嵌入下一条`BESTPOSA`。原始字节流仍完整保存，因此这被判定为接收机输出完整性异常，而非录制器丢失数据。

### 正式测试前的下一步

1. 在目标 BX100Y/TX100Y 按正式 COM1 配置录制 2–3 分钟预检日志。
2. 用离线工具确认实际 `BESTPOSA`/`UNIHEADINGA` 字段、CRC、端口头、频率、带宽和差分状态。
3. 预检通过后再执行静态 3 分钟、局部测试网、公里级闭环和通信中断测试；配置变更或首次部署时，追加 10 分钟串口压力检查。
4. 扩展离线分析，完成静态重复性、局部距离/闭环、事件对齐和航向统计；航向必须分开处理 `NARROW_INT` 与 `NARROW_FLOAT`，并验证角度方向与安装零偏。

## 文档

| 文档 | 内容 |
|---|---|
| [外场测试方案](docs/RTK_TEST_PLAN.md) | 测试边界、正式 COM1 输出、静态点、局部测试网、闭环、电台事件和现场记录模板。 |
| 本 README 的“已验证数据边界” | 两份已有非正式日志的适用范围与限制。 |

## 工具与运行环境

| 工具 | 运行位置 | 用途 |
|---|---|---|
| `scripts/serial_capture.py` | 低权限 RK3588 | 保存原始串口字节流与逐行主机时间索引。 |
| `bx100_test_recorder` | 开发机 | 发布 ROS 2 话题并保存带主机接收时间的原始串口行。 |
| `scripts/rtk_offline_analysis.py` | 开发机 | 清点消息、校验帧、导出 GGA/ENU 初步结果。 |
| `scripts/rtk_amap_view.py` | 开发机 | 将校验通过的 GGA 原始轨迹导出为高德地图 HTML。 |

## RK3588 低权限采集

从工作区根目录复制录制器：

```bash
scp rtk_accuracy_test/scripts/serial_capture.py <user>@<rk3588_ip>:/home/<user>/
```

在 RK3588 上确认串口后录制预检日志：

```bash
mkdir -p ~/rtk_data/
python3 ~/serial_capture.py \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --duration 180 \
  --raw-output ~/rtk_data/rover.raw \
  --index-output ~/rtk_data/rover.tsv
```

持续录制时省略 `--duration`，按 `Ctrl-C` 停止。脚本生成完整字节流 `.raw` 与逐行主机接收时间索引 `.tsv`；接收机 UTC 仍在 `GGA`、`ZDA`、`TIMEA` 等原始消息中。

取回数据时保留整个会话目录：

```bash
scp -r <user>@<rk3588_ip>:/home/<user>/rtk_data/preflight_001 ./data/
```

## ROS 2 构建与移动站录制

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rtk_accuracy_test --symlink-install
source install/setup.bash
```

将接收机配置为在实际采集端口输出 COM1 日志；同一时间只连接移动站或基准站中的一台：

```bash
ros2 run rtk_accuracy_test bx100_test_recorder --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud_rate:=115200 \
  -p log_directory:=./data/
```

节点保存以角色和时间命名的 `rover_*.raw`，每行格式为 Unix 主机接收时间（ns）、制表符和原始串口行。基准站检查应在移动站测试前后分时进行，分别记录 `base_before.raw` 和`base_after.raw`；动态测试期间不可重新自主优化基准站。

## 离线预检、初步分析与地图回放

先清点目标机预检日志：

```bash
python3 scripts/rtk_offline_analysis.py data/rover_*.raw \
  --origin LAT LON HEIGHT \
  --csv data/analysis/gga_enu.csv \
  --summary data/analysis/summary.txt
```

该工具当前输出 GGA/ENU 行、解状态计数、消息清点和 CRC 统计；它不是完整的外场精度统计工具。静态窗口、测试网距离、事件对齐和闭环统计要等目标机 `BESTPOSA` 字段顺序和状态枚举通过预检后再固化。

导出轨迹地图：

```bash
python3 scripts/rtk_amap_view.py \
  data/rover.raw \
  data/analysis/rover_amap.html
```

页面只绘制 NMEA 校验和正确且经纬度完整的 GGA 样本，按 GGA 解状态分色并在状态变化处断线。浏览器中输入高德 Web JS API Key 后加载底图；WGS-84 坐标会转为 GCJ-02 用于绘制。起终点信息窗保留原始 WGS-84 坐标、高程、GGA UTC、解状态和可用的主机接收时间。

可选的 `--amap-key` 与 `--security-js-code` 会把凭据明文写入 HTML；不要分享或提交带有这些参数生成的文件。默认的浏览器输入只保存在该浏览器的 `localStorage`。

## 数据与提交约定

原始串口数据、rosbag、构建目录、日志、IDE 配置、压缩包和密钥文件均已由 `.gitignore`排除。会话目录不可覆盖，正式执行和报告口径以 [外场测试方案](docs/RTK_TEST_PLAN.md) 为准。提交前在本仓库内检查：

```bash
git status --short
git diff --cached --stat
```
