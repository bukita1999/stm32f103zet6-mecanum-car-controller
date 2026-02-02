# Sequence Create 不启动串口 设计

## 目标
`sequence --create` 仅用于交互式生成 CSV，不启动遥测线程与串口客户端。

## 方案
在 `cli.py` 中检测 `args.mode == "sequence"` 且 `args.create` 为真时：
- 调用 `create_sequence_csv_interactive(_default_data_dir())` 生成 CSV。
- 成功后直接返回 `0`，失败则记录错误并返回 `1`。
- 该分支位于 `TelemetryLogger` 与 `SerialCommandClient` 初始化之前，确保不读写串口。

## 影响范围
- `sequence --csv/--tui` 保持原有逻辑。
- `remote/webdebug` 不受影响。
- README 补充说明该模式不会启动串口或遥测。
