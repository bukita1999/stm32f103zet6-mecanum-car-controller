# Sequence Create TUI 设计

## 目标
新增一个 `sequence --create` 的文本交互式流程，用于生成符合现有 `data/*.csv` 格式的序列文件。该流程需支持目录选择、文件命名、节点输入、线性插值与严格校验，且不引入新依赖。

## 交互流程
1. 进入 `sequence --create` 后，限制在 `python/local_controller/data/` 及其子目录内选择输出目录。
2. 输入文件名（自动补 `.csv`），若文件已存在则询问是否覆盖（y/n）。
3. 输入 `step_ms`（默认 500），用于插值间隔。
4. 自动写入固定两行：`0ms: 0,0,0,0` 与 `1000ms: 0,0,0,0`，并将 1000ms 设为上一节点。
5. 循环输入下一个时间点（毫秒整数，必须递增且为 `step_ms` 的整数倍），再输入四个轮速（整数）。输入 `q/quit` 结束并保存。

## 校验规则
- 速度输入必须是整数；若任一非整数，提示错误并重试。
- 若某个轮速非 0，则必须满足 `m0 < 0、m1 > 0、m2 > 0、m3 < 0`；若为 0，则该轮不做正负限制。
- 时间点必须严格递增，且是 `step_ms` 的整数倍；否则提示错误并重试。

## 插值规则
- 只生成区间内部点：`(t_prev, t_next)`，不包含两端。
- 使用线性插值并对结果做“0.5 进位”的四舍五入。
- 每段插值输出 `t_prev + step_ms` 到 `t_next - step_ms` 的点。

## 代码位置
- CLI 入口：`python/local_controller/local_controller/cli.py` 增加 `sequence --create` 选项并触发生成流程。
- 生成逻辑：`python/local_controller/local_controller/modes/sequence.py` 新增目录选择、输入校验、插值与 CSV 写入函数。
- 文档说明：更新 `python/local_controller/README.md`。

## 测试与校验
- 静态检查：运行 `mypy --strict` 覆盖新增逻辑。
- 手动验证：生成文件后核对头部、固定行、插值点与符号规则。
