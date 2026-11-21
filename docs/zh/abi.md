# ABI 指南（`motor_abi`）

## 构建

```bash
cargo build -p motor_abi --release
```

产物：

- Linux：`target/release/libmotor_abi.so`、`libmotor_abi.a`
- Windows：`target/release/motor_abi.dll`、`motor_abi.lib`
- 头文件：`motor_abi/include/motor_abi.h`

## 统一接口面

ABI 对外保持一套统一控制接口。

## ABI 元数据

- `motor_abi_version()` 返回当前加载的 ABI 库版本字符串。
- `motor_abi_capabilities_json()` 返回稳定 JSON 能力文档，描述已支持的
  transports、vendors、controller lifecycle、control modes 和厂商专用 features。
- Python 中对应 `motorbridge.abi_version()` 和 `motorbridge.abi_capabilities()`。
- C++ 中对应 `motorbridge::abi_version()` 和 `motorbridge::abi_capabilities_json()`。
- `bindings/api_surface.json` 用于跟踪 ABI、Python、C++ 和文档的接口对齐。

统一模式 ID（`motor_handle_ensure_mode`）：

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

统一控制单位：

- 位置：`rad`
- 速度：`rad/s`
- 力矩：`Nm`

通用控制/状态接口：

- `motor_handle_enable`
- `motor_handle_disable`
- `motor_handle_clear_error`
- `motor_handle_set_zero_position`
- `motor_handle_ensure_mode`
- `motor_handle_send_mit`
- `motor_handle_send_pos_vel`
- `motor_handle_send_vel`
- `motor_handle_send_force_pos`
- `motor_handle_request_feedback`
- `motor_handle_set_can_timeout_ms`
- `motor_handle_store_parameters`
- `motor_handle_get_state`
- `motor_handle_get_state_with_age`（Damiao）

`motor_handle_get_state_with_age(motor, &state, &age_us, &seq)` 在同一把锁下返回缓存的状态、
其年龄（状态帧到达后经过的微秒数）和序号（电机加入后收到的状态帧数）。Damiao 电机对每个控制帧
（MIT、pos-vel 等）和每个反馈请求都回复状态帧，且都写入同一缓存；因此每周期都发送控制帧的调用方
可以不再发 `motor_handle_request_feedback`，改为检查年龄。与缓存寄存器读取函数一样有三种返回值，
不要用 `rc != 0` 或 `rc < 0` 判断：`0`（已写入状态、年龄和序号）、`1`（尚无状态帧；`state` 以
`has_value = 0` 写入，其余不写）、`-1`（错误：空指针或非 Damiao 电机）。能力标志：`state_age`。年龄从帧被处理（写入缓存）时算起：后台轮询线程与
`motor_handle_poll_feedback_once` 同时读取总线时，同一电机的旧帧可能在新帧之后被处理，因此状态可能比
其年龄所示早一帧（但年龄大于某时刻仍表示此后没有处理过任何状态帧）。

## 厂商入口

- Damiao：`motor_controller_add_damiao_motor(...)`
- Hexfellow：`motor_controller_add_hexfellow_motor(...)`（通过 `socketcanfd` 走 CAN-FD）
- RobStride：`motor_controller_add_robstride_motor(...)`
- MyActuator：`motor_controller_add_myactuator_motor(...)`
- HighTorque：`motor_controller_add_hightorque_motor(...)`

## 统一模式与厂商原生协议映射

| 统一模式 | Damiao 原生 | Hexfellow 原生 | RobStride 原生 | MyActuator 原生 | HighTorque 原生 |
|---|---|---|---|---|---|
| `MIT` | `Mit` | 模式 `5` | `Mit` | 不支持 | 映射到原生 pos+vel+tqe |
| `POS_VEL` | `PosVel` | 模式 `1` | 映射到 `Position`（`run_mode=1`，`limit_spd=0x7017`，`loc_ref=0x7016`） | `Position` 设定流程 | 映射到原生 pos+vel+tqe |
| `VEL` | `Vel` | 不支持 | `Velocity` | `Velocity` 设定流程 | 映射到原生速度命令 |
| `FORCE_POS` | `ForcePos` | 不支持 | 不支持 | 不支持 | 映射到原生 pos+vel+tqe |

行为约定：

- 不支持的调用返回非 0，并可通过 `motor_last_error_message()` 获取清晰错误信息。
- 即使某厂商忽略部分参数，也保持统一函数签名不变。
- 例如：HighTorque 支持 `send_mit(pos, vel, kp, kd, tau)` 统一签名，但原生协议不使用 `kp/kd`。
- Hexfellow 的 ABI 路径支持 `MIT` 和 `POS_VEL`，`VEL` / `FORCE_POS` 会返回不支持。
- RobStride 的 ABI 路径支持 `POS_VEL`，语义映射为原生 Position：先设置 `run_mode=1`，再写 `limit_spd` 与 `loc_ref`。
- RobStride 的统一高层目前支持 `MIT/POS_VEL/VEL`；`TORQUE/CURRENT` 仍是参数级能力（通过 `robstride_write_param_*`），尚未开放统一模式。
- Damiao 置零顺序规则：先调用 `motor_handle_disable`，再调用 `motor_handle_set_zero_position`；否则会被核心防护拒绝。
- Damiao 置零稳定规则：`set_zero_position` 成功后，核心层内置固定稳定等待（约 `20ms`），ABI 不额外暴露等待参数。

## 厂商扩展接口

Damiao 寄存器接口：

- `motor_handle_write_register_f32`
- `motor_handle_write_register_u32`
- `motor_handle_get_register_f32`
- `motor_handle_get_register_u32`
- `motor_handle_request_register`
- `motor_handle_get_cached_register_f32`
- `motor_handle_get_cached_register_u32`

非阻塞寄存器读取：先对每个电机调用 `motor_handle_request_register`，再轮询反馈
（`motor_controller_poll_feedback_once`），然后调用
`motor_handle_get_cached_register_f32/u32(motor, rid, &value, &age_us, &fresh)`。
与其他 ABI 调用不同，这两个函数有三种返回值，请勿用 `rc != 0` 或 `rc < 0` 判断：

- `0`：已有缓存回复，写入 `value`、`age_us`（回复到达至今的微秒数）和 `fresh`。
  `fresh` 为 1 表示该回复对应最近一次请求，为 0 表示是更早请求的残留（例如最近
  一次请求丢失）。将 `value` 视为当前值之前请先检查 `fresh`。
- `1`：该寄存器从未收到回复，不写入任何值。
- `-1`：错误（空指针、非 Damiao 电机或寄存器类型不符）。

RobStride 扩展接口：

- `motor_handle_robstride_ping`
- `motor_handle_robstride_ping_host_id`
- `motor_handle_robstride_set_device_id`
- `motor_handle_robstride_set_active_report`
- `motor_handle_robstride_get_fault_report`
- `motor_handle_robstride_get_param_f32_host_id`
- `motor_handle_robstride_write_param_i8/u8/u16/u32/f32`
- `motor_handle_robstride_get_param_i8/u8/u16/u32/f32`

## 典型调用顺序

1. 选择传输层构造器：
   - `motor_controller_new_socketcan(channel)`（通用路径）
   - `motor_controller_new_socketcanfd(channel)`（CAN-FD 路径；Hexfellow 必须使用）
   - `motor_controller_new_dm_serial(serial_port, baud)`（仅 Damiao 串口桥；跨平台，可用 `/dev/ttyACM0` 或 `COM3`）
2. `motor_controller_add_<vendor>_motor`
3. 可选：`motor_controller_enable_all`
4. 可选：`motor_handle_ensure_mode`
5. 发送控制命令 / 读取状态 / 调用厂商扩展接口
6. `motor_controller_shutdown`
   - 或在只关闭本地会话/总线时调用 `motor_controller_close_bus`
7. `motor_handle_free`
8. `motor_controller_free`

## 示例

- C ABI 示例：`examples/c/c_abi_demo.c`
- C++ ABI 示例：`examples/cpp/cpp_abi_demo.cpp`
- Python ctypes 示例：`examples/python/python_ctypes_demo.py`
