# REMOTE MOTOR CONTROLLER - BLE CONNECTIVITY SUBSYSTEM (STM32WB + Zephyr)

This firmware provides the **Bluetooth Low Energy (BLE) connectivity** for a remote-controlled motor system. It provides a custom GATT service for **COMMANDS** (phone -> device) and **telemetry** (device -> phone) and is designed to stay decoupled.

> MCU/OS: STM32(WB55), Zephyr RTOS

---

## FEATURES

- Connectable advertising with a **128-bit service UUID**
- Custom GATT
    - **COMMAND* characteristic (Write): drive mode/target for Motor
    - **Telemetry** characteristic (Notify): status/speed/position
    - CCC to enable/disable notifications
    - Little-endian framework for the payloads

---

## GATT MODEL
**Service UUID**: `c52081ba-e90f-40e4-a99f-ccaa4fd11c15`

| Characteristic | UUID                                   | Props        | Value                                |
|----------------|----------------------------------------|--------------|--------------------------------------|
| Command        | `d10b46cd-412a-4d15-a7bb-092a329eed46` | Write        | `[1B cmd][4B value_le]`              |
| Telemetry      | `17da15e5-05b1-42df-8d9d-d7645d6d9293` | Notify (+R)  | `[1B status][4B speed][4B pos_deg]`  |

> CCC (0x2902) follows Telemetry value. You can add **Characteristic User Description** (0x2901) to show human-readable names in generic scanner apps.

---

## Protocol

All multi-byte values are **little-endian**.

**Command write** (`len=5`)
[0] cmd:
0x00 = SHUTDOWN
0X01 = INIT
0X02 = SET_SPEED (rpm in [1..4])
0X03 = SET_POSITION (degree in [1..4])

[1..4] value_le: int32

**Telemetry Notify** ('len=9')
[0] status : bitfield (0x01=OK, 0x02=FAULT, 0x00=STOP)
[1..4] speed_le: int32 rpm
[5..8] post_le: int32 degrees (0..359)

