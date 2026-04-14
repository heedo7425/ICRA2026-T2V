# ICRA2026-T2V

T2V (Transmitter to Vehicle) IR remote control system for the ICRA 2026 RoboRacer competition.

Uses two ESP32-S3 Super Mini boards to transmit START/STOP commands to the vehicle via NEC IR signals,
and bridges the commands to ROS1 topics for integration with the autonomous racing stack.

## Structure

```
t2v_transmitter/   # ESP32-S3 IR transmitter (NEC 38kHz + BLE GATT)
t2v_receiver/      # ESP32-S3 IR receiver (NEC decode + USB CDC + BLE GATT)
t2v_node/          # ROS1 package (USB CDC → /joy topic bridge)
```

## Data Flow

```
Transmitter key (1/2/3) → NEC IR 38kHz → Receiver VS1838B
                                              ↓
                                      NEC decode + USB CDC
                                              ↓
                                      t2v_bridge.py (ROS1)
                                              ↓
                                      /joy (sensor_msgs/Joy)
                                              ↓
                                      simple_mux.py (racing stack)
                                              ↓
                                      Autonomous start / stop
```

## Commands

| Key | IR Code | Joy Mapping | Action |
|-----|---------|-------------|--------|
| 1 | START_GO (0x02) | buttons[5] (RB) | Start autonomous mode |
| 2 | STOP (0x7F) | buttons[4] (LB) + axes[1]=0 | Stop |
| 3 | START_ABORT (0x03) | buttons[4] (LB) + axes[1]=0 | Emergency stop |

## Build

### ESP32 Firmware (ESP-IDF v5.3)

```bash
# Transmitter
cd t2v_transmitter
idf.py build
idf.py -p /dev/ttyACM1 flash monitor

# Receiver
cd t2v_receiver
idf.py build
# Enter download mode with BOOT+RST, then:
idf.py -p /dev/ttyACM0 flash
```

### ROS1 Node (Noetic)

```bash
# Place t2v_node under your catkin workspace, then:
catkin_make
rosrun t2v_node t2v_bridge.py
```

## Hardware

| Item | Transmitter | Receiver |
|------|-------------|----------|
| Board | ESP32-S3 Super Mini | ESP32-S3 Super Mini |
| IR | GPIO5 (IR LED) | GPIO6 (VS1838B) |
| USB | USB Serial/JTAG | TinyUSB CDC (VID:0x5455 PID:0x1911) |
| BLE | T2V-Transmitter | T2V-Receiver |

## Documentation

- [T2V_CHECKPOINT.md](T2V_CHECKPOINT.md) — Full project specification
- [T2V_INTEGRATION.md](T2V_INTEGRATION.md) — Racing stack integration guide

## Team

UNICORN Racing — ICRA 2026 RoboRacer
