# Local Controller

CLI 工具用于通过 UART/USB 控制 STM32 麦克纳姆小车，并记录 USB-CDC 遥测二进制帧。

## 依赖

- Python 3.13（`uv` 已在仓库中生成虚拟环境）
- PySerial（`uv add pyserial` 已添加）

首次使用可运行：

```bash
cd python/local_controller
uv run python -m local_controller --help
```

## 配置

`config.toml` 保存串口与遥控配置：

```toml
[serial.command]
port = "/dev/ttyUSB0"
baudrate = 115200

[serial.telemetry]
port = "/dev/ttyACM0"
baudrate = 115200

[remote.speed_profiles]
forward = [-4000, 4000, 4000, -4000]
backward = [4000, -4000, -4000, 4000]
left = [-2000, 1500, 4500, -5000]
right = [-4500, 5000, 2000, -1500]
stop = [0, 0, 0, 0]
```

- `serial.command`: 用于发送 `$SPD` 指令的 UART 端口。
- `serial.telemetry`: 用于接收 USB CDC 二进制帧的端口；日志写入 `output/telemetry_*.csv`。
- `remote.speed_profiles`: 遥控模式的方向速度表，可按需扩展。

## 串口命令速查

使用 `SerialCommandClient.send_speeds` 同样的格式（`$SPD,<m0>,<m1>,<m2>,<m3>#`），基于当前 `config.toml`：

- 前进：`$SPD,-4000,4000,4000,-4000#`
- 后退：`$SPD,4000,-4000,-4000,4000#`
- 左转：`$SPD,-2000,1500,4500,-5000#`
- 右转：`$SPD,-4500,5000,2000,-1500#`
- 停止：`$SPD,0,0,0,0#`

在新建的 `webdebug` 页面或任意串口终端中直接粘贴整行即可发送；如果修改了 `config.toml` 中的速度，按同样格式替换相应数值即可。

## 遥控模式

交互式遥控：

```bash
uv run python -m local_controller remote
```

- `w/s/a/d/x` 对应 前进/后退/左转/右转/停止，`q` 退出。
- 也可直接发送一次动作：

```bash
uv run python -m local_controller remote --action forward
```

## 预定义序列

CSV 文件包含 `time_s,m0,m1,m2,m3` 列，例如 `data/sample_sequence.csv`：

```csv
time_s,m0,m1,m2,m3
0.0,0,0,0,0
2.0,-4000,4000,4000,-4000
4.5,-2000,1500,4500,-5000
7.0,0,0,0,0
9.0,4000,-4000,-4000,4000
12.0,0,0,0,0
```

运行：

```bash
uv run python -m local_controller sequence --csv data/sample_sequence.csv
```

脚本会按 `time_s` 时间戳顺序发送速度命令。

## 遥测记录

无论哪种模式，`TelemetryLogger` 都会：

- 在 `output/<mode>_<timestamp>.csv` 中写入解码后的帧（`<mode>` 为 `remote/sequence/webdebug`）。
- CSV 字段：`host_time,frame_timestamp_ms,motor_id,target_rpm,current_rpm,pwm_percent`。

终止程序或 `Ctrl+C` 时会自动停止遥测线程并关闭串口，随后询问是否保留 CSV。
