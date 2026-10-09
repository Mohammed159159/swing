# Hardware Telemetry & HIL Communication Protocol Contract (`hardware/firmware/protocol/`)

This directory documents and defines the bi-directional serial packet contract between the embedded microcontroller and the MATLAB App Designer / telemetry bridge.

---

## 📡 Packet Structure Specification (HANDBOOK.md §9)

To ensure low latency and high bandwidth, telemetry is formatted as fixed-size packed binary structs rather than plain ASCII strings.

### 1. Downlink Packet: Microcontroller $\to$ MATLAB (Streamed at 500 Hz)

```text
[0xAA] [0x55] [Timestamp: uint32] [theta0: float32] [theta1: float32] [theta0_dot: float32] [theta1_dot: float32] [voltage: float32] [current: float32] [state_id: uint8] [CRC16: uint16]
```

| Offset (Bytes) | Field Name | Data Type | Units / Description |
|---|---|---|---|
| 0–1 | `sync_header` | `uint8[2]` | Preamble magic bytes: `0xAA, 0x55` |
| 2–5 | `timestamp_ms`| `uint32` | Milliseconds since MCU startup |
| 6–9 | `theta0` | `float32` | Arm angle $[\text{rad}]$ |
| 10–13 | `theta1` | `float32` | Pendulum angle $[\text{rad}]$ |
| 14–17 | `theta0_dot` | `float32` | Arm angular velocity $[\text{rad/s}]$ |
| 18–21 | `theta1_dot` | `float32` | Pendulum angular velocity $[\text{rad/s}]$ |
| 22–25 | `voltage` | `float32` | Motor output command $[\text{V}]$ |
| 26–29 | `current` | `float32` | Motor current reading $[A]$ |
| 30 | `state_id` | `uint8` | Active state (`0: SAFE_STOP`, `1: SWING_UP`, `2: STABILIZE`) |
| 31–32 | `checksum` | `uint16` | CRC-16-CCITT across bytes 2–30 |

*Total Packet Size:* 33 Bytes per frame $\implies 33 \times 500 = 16.5\text{ kB/s}$ (well within $115200\text{ baud}$ throughput, highly optimal at $460800\text{ baud}$).

---

### 2. Uplink Packet: MATLAB $\to$ Microcontroller (Command / Tuning)

```text
[0x55] [0xAA] [cmd_id: uint8] [param1: float32] [param2: float32] [CRC16: uint16]
```

- `cmd_id = 0x01`: Emergency Stop latch (`param1 = 0`).
- `cmd_id = 0x02`: Switch active controller (`0: PID`, `1: PP`, `2: LQR`, `3: RL`).
- `cmd_id = 0x03`: Set arm target reference $\theta_{0,\text{ref}}$.
- `cmd_id = 0x04`: Online gain update.
