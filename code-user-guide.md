# 云台固件使用指南

本文按当前源码说明固件的遥控操作、状态切换、调参和检查方式。实际运行行为以 `src/main.rs`、`src/dm_mc02.rs`、`src/pid.rs` 和 `src/fault.rs` 为准；如果参数或开关被修改，应同步更新本指南。

## 1. 上电前检查

这是会通过 CAN 向电机发送电压指令的固件。首次上电、修改控制参数或改变机械安装后：

1. 先确认云台周围没有人员或障碍物，机械活动范围安全，并准备好使用遥控器停止。
2. 核对电机 ID、CAN 接线、反馈方向及机械安装。代码中 yaw 使用反馈 ID `0x205`，pitch 使用 `0x206`；组电压命令 ID 为 `0x1FF`。
3. 先在不使能输出时确认两轴反馈方向、角度和转速读数合理。编码中的正方向不保证就是从任意观察角度看到的逆时针方向，须结合实际安装确认。
4. 确认遥控器接收正常。有效 DBUS 帧包含 5 个中心化通道（范围 `-660..=660`）和两个拨杆状态；无效帧会作为无遥控输入处理。
5. 确认有办法让电机进入零指令状态。固件通过 CAN 发送零指令，但 CAN 队列或总线异常时，不能保证电机已经收到停止命令。

**不要把固件的输出限幅、故障检测或遥控停止视为硬件急停。** 先使用保守参数和低风险环境做验证；不要用手强行卡住电机，也不要故意制造剧烈振荡。

## 2. 构建和启动

项目根目录的 `.cargo/config.toml` 已设置目标三元组 `thumbv7em-none-eabihf`、芯片 `STM32H723VGTx` 对应的 probe-rs runner，以及 Cortex-M7 和链接脚本参数。根目录 `Cargo.toml` 的 release profile 保留调试信息，供 Tuning Studio 定位变量。

命令行检查和构建：

```sh
cargo check --release --locked
cargo build --release --locked
```

使用 probe-rs runner 构建、烧录并查看 RTT：

```sh
cargo run --release --locked
```

也可以在 VS Code 使用 Tuning Studio：

1. 停止其他占用调试探针的 probe-rs 或 VS Code 调试会话。
2. 在工作区打开 Tuning Studio 侧栏，点击 **Run Firmware**。
3. 扩展会按 `.vscode/settings.json` 中的 `--release --locked` 参数构建，并通过 Run 时覆盖的 Cargo runner 烧录和启动会话。
4. RTT 输出显示在扩展启动的终端中；Live Watch 用于查看遥测及调节已暴露的 PID 增益。

手动打开 ELF / 连接目标时，ELF 是编译产物（通常为 `target/thumbv7em-none-eabihf/release/control-training-template`），包含代码和调试符号；Connect Target 则选择实际调试探针及芯片。它们不能代替烧录。不要让 Tuning Studio 和另一个调试会话同时占用同一个探针。

结束主机端调试/监视会话不等于给电机发送 disable 指令。应通过遥控器切换到等待状态，并确认输出和设备状态；如总线失效，使用设备规定的独立安全断电方式。

## 3. 遥控器通道

当前使用**左摇杆**控制云台两轴；右摇杆没有接入 yaw/pitch 控制。

| DR16 数据 | 当前用途 | 代码处理 |
|---|---|---|
| `channels[2]`，左摇杆左右 | yaw 目标角度 | 减去 `YAW_CHANNEL_OFFSET`（当前为 `100`），限幅到 `-660..=660`，再除以 `660` 并经过死区 |
| `channels[3]`，左摇杆上下 | pitch 目标角度 | 除以 `660` 并经过死区；当前没有额外 offset |
| `channels[0]`、`channels[1]`，右摇杆 | 未用于两轴控制 | 仅可在遥控器日志中观察 |
| `right_switch` | 模式与 enable/disable | `1` 上位、`2` 下位、`3` 中位 |
| `left_switch` | 故障恢复辅助动作 | 仅在 `Error` 状态按下文条件使用 |

`dm_mc02.rs` 解包 DBUS 时已把每个原始通道减去 `1024`，所以 `channels[2]` 是中心化后的值。若原始静止读数比 `1024` 高约 `100`，此时 `channels[2]` 约为 `+100`；`YAW_CHANNEL_OFFSET = 100` 再将静止输入校正到约 `0`。请以 RTT 实际静止读数校准该常量。这个 offset 校正会使摇杆正负两侧剩余行程不对称。

两轴均使用 `0.05` 死区。死区外输入会重新映射到 `0..1` 的完整幅度。Running 中，摇杆不是直接输出转速或电压，而是持续移动目标角度：

- yaw 的最大目标角变化率当前为 `20 rad/s`。
- pitch 的最大目标角变化率当前为 `2 rad/s`。
- 控制周期目标为 `1 ms`。推住摇杆时目标角持续变化；摇杆回中后目标角停止变化，位置环继续保持目标。
- yaw 目标按单圈角度保持在 `[0, 2π)`；pitch 目标限制在 `PITCH_CALIB_ANGLE_RAD + PITCH_MIN_ANGLE_RAD` 与 `PITCH_CALIB_ANGLE_RAD + PITCH_MAX_ANGLE_RAD` 之间（当前约 `2.80..=3.50 rad`），不做单圈回绕。两个 pitch 限值应设在机械硬限位以内，预留安全余量。
- 位置误差使用最短方向修正，范围约为 `[-π, π]`。

摇杆和电机的实际方向要按教具安装校准。若方向与预期相反，应停止输出并检查通道/电机方向配置，不要在高速运行时尝试纠正。

## 4. 右拨杆状态操作

右拨杆值来自 `remote.right_switch`：`1` 上位、`2` 下位、`3` 中位。当前控制器初始状态是 `Waiting`。

| 右拨杆操作 | 状态行为 |
|---|---|
| 下位 `2` | 进入 `Waiting` 并设置一次使能准备标记。可将其作为启动前的明确停用/准备动作。 |
| 下位 `2` 后拨中位 `3` 并保持 | 若仍在 `Waiting` 且反馈有效，中位保持满 `1 s` 后进入 `Prepare`。 |
| `Prepare` | 以 yaw `2.5564 rad`、pitch `3.1000 rad` 为回正目标，使用位置环和速度环；两轴角误差均不超过 `0.1 rad` 且转速均不超过 `6 rpm`，连续满足 `100 ms` 后自动进入 `Running`。 |
| 上位 `1` | 在 `Waiting` 且先前有下位准备标记时进入 `Normal`；在 `Prepare` 或 `Running` 中则回到 `Waiting`。它不是从 `Waiting` 直接启动回正的操作。 |
| 中位 `3` | 在 `Normal` 中回到 `Waiting`；在 `Waiting` 中只有先经下位准备并持续满 `1 s` 才启动 `Prepare`。 |
| 非法/没有有效遥控输入 | 回到 `Waiting`，清除使能准备状态。 |

**当前重要配置：** `PREPARE_OUTPUT_ENABLED = true`，`RUNNING_OUTPUT_ENABLED = true`，但 `NORMAL_OUTPUT_ENABLED = false`。所以 Normal 会计算两轴 `30 rpm` 的速度环指令，但 transmit 不会把该模式的电压指令放进 CAN 帧；不要把 Normal 当作当前可驱动电机的模式。

建议的一般启动流程：

1. 确认反馈和遥控输入正常。
2. 右拨杆先下位 `2`。
3. 若要回正并进入摇杆控制，将右拨杆拨到中位 `3` 并保持至少 `1 s`。
4. 等待 `Prepare` 自动完成，确认进入 `Running` 后再轻推左摇杆。
5. 退出 `Running` 可将右拨杆拨到上位 `1` 或下位 `2`；状态回到 `Waiting`，电压指令清零。

输入或反馈失效会阻止需要闭环反馈的运行；Normal、Prepare、Running 都检查 yaw/pitch 两轴反馈是否在 `100 ms` 内更新。遥控输入连续 `100` 个控制周期（约 `100 ms`）无有效帧时会被清除，状态回到 `Waiting`。

## 5. PID、输出和实时调参

Running 和 Prepare 使用串级双环：

1. 位置环输入目标角度及电机角度，计算目标转速，单位为 `rpm`，输出受各自位置 PID 的 `maxout` 限制。
2. 速度环比较目标转速与反馈转速，计算 CAN 电压命令（协议原始值）。
3. 电压命令还会受 `MAX_VOLTAGE_COMMAND = 25000` 的编码限幅。

PID 积分和 D 项按**每次调用**更新，而不是按秒计算。控制频率改变后，增益含义也改变，需重新评估和调参。调参请一次只改少量参数，先安全停机/低输出验证方向，再逐步增加。

### Tuning Studio

控制任务每周期通过 `watch_mut!` 暴露四组 PID 的 `kp`、`ki`、`kd`，可在 Live Watch 中编辑：

- `yaw.position.kp` / `.ki` / `.kd`
- `yaw.speed.kp` / `.ki` / `.kd`
- `pitch.position.kp` / `.ki` / `.kd`
- `pitch.speed.kp` / `.ki` / `.kd`

修改通过固件的可变引用在下一控制周期应用。当前使用 `watch_mut!`，**不提供范围或步长限制，也不保存到 Flash**；固件重启后恢复默认值。建议只在设备处于安全状态时输入有限、保守的值。

Live Watch 还发布以下只读值，可加入 Plot：

- yaw/pitch：`*.angle.target`、`*.angle.measured`、`*.angle.error`
- yaw/pitch：`*.speed.target`、`*.speed.measured`
- yaw/pitch：`*.voltage.command`

主机轮询采样低于 1 kHz 控制循环；默认曲线采样约 `100 Hz`，不能据此判断没有更快的抖动或瞬态。

### RTT PID 命令

也可在 RTT **Down Channel 0** 输入命令，以换行结束：

```text
help
show
set yaw speed <kp> <ki> <kd> <maxout> <integral_limit>
```

`set` 例子（具体数值只作格式示例，不是推荐调参值）：

```text
set yaw speed 80 0.05 0 25000 12500000
```

支持的轴为 `yaw`、`pitch`，控制环为 `position`、`speed`。成功修改会清除此 PID 的积分和误差历史；参数仅在本次运行中有效。Tuning Studio 与 RTT 都能改同一组 `kp/ki/kd`，调参时不要同时从两种接口写同一组参数，以免后写入的值覆盖先前设置。RTT 命令还可修改限幅；Tuning Studio 当前未暴露限幅字段。

## 6. 故障、停止与恢复

`Waiting` 和 `Error` 状态会清零内存中的两轴电压命令；transmit 会尝试发送零值 CAN 组帧。CAN 写队列满会记录 `tx_failed` 并打印日志，但无法保证停止帧送达。

当前实现中的检测开关：

| 检测 | 当前是否启用 | 代码中的判据（启用时） |
|---|---|---|
| 堵转 | yaw、pitch 分别配置，目前两轴都关闭 | 每轴独立配置输出使能、最小目标转速、最小电压命令、最大实际转速和持续时间。当前两轴配置均为 `5 rpm`、`80`、`1 rpm`、`3000 ms`，但 `enabled = false` |
| 持续高频振荡 | 是 | 速度方向反复变化；振幅至少 `0.02 rad`、周期不超过 `333 ms`，持续 `500 ms` |

因此**当前固件会检测持续高频振荡，但不会自动检测堵转**。代码还会在以下情形进入 `Error`：

- Normal 状态控制循环间隔超过约 `2 ms`，触发 `ControlTiming`。
- Prepare、Running 或 Normal 中 PID 输入/配置/计算结果无效，触发 `InvalidControl`。

反馈超时会回到 `Waiting`（若已在 `Error`，则保留 Error）。CAN 发送队列失败目前计数并记录日志，不会单独触发 Error。`PrepareTimeout`、`NormalOverspeed` 等枚举在当前路径没有启用的触发逻辑。

故障恢复需要同时满足：

1. 状态已经是 `Error`。
2. 右拨杆保持中位 `3`。
3. 左摇杆左右、上下均回到死区。
4. 左拨杆在进入 Error 后先处于下位 `2`，再切到上位 `1`，形成一次新的下到上动作。

恢复会清除故障、PID 历史和故障监视计数，把两轴角度目标设为当前反馈角，并直接回到 `Running`。这会恢复 Running 输出；确认机械和电机条件安全后才执行。若故障条件仍然存在且相应检测已启用，可能再次触发。

## 7. 遥测和问题排查

RTT 上行日志每约 `500 ms` 输出一次状态及遥控数据。重点字段：

| 字段 | 含义 |
|---|---|
| `state`、`fault` | 控制状态和已记录故障原因 |
| `yaw_angle`、`yaw_error` | yaw 单圈反馈角和环绕修正后的目标角误差（rad） |
| `yaw_rpm`、`yaw_target` | yaw 实际转速与位置环目标转速（rpm） |
| `yaw_voltage` | yaw 电压命令协议原始值 |
| `tx_failed` | CAN 发送队列写入失败累计次数 |
| `channels` | 已减去 1024 的中心化遥控通道值 |
| `raw_channels` | 由中心化值加回 1024 显示的原始通道近似值 |

RTT 日志采样较慢，可能错过瞬间变化。分析“卡顿/弹动”时，同时看目标速度、实际速度、角度误差和电压命令；若需高频诊断，应采用安全的专用记录方法，不能仅凭低频曲线下结论。

## 8. 重要实现边界

- 电机速度和电流反馈的符号/单位仍需按实际电机协议和教具验证。
- `Motor::angle_rad` 是电机原始单圈角度，不累计圈数；云台机械限位、线缆绕转限制不由单圈环绕逻辑自动保证。
- yaw offset 默认 `100`，需按当前遥控器静止数据重新校准；错误 offset 会造成摇杆静止时目标漂移。
- 输出限幅是软件命令限幅，不等同于电流限制、机械保护或硬件急停。
- 运行结束、关闭 VS Code 或断开探针不会自动保证电机停止；必须确认遥控停用动作或使用独立硬件安全措施。