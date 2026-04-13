# ICRA2026-T2V

ICRA 2026 RoboRacer 대회용 T2V (Transmitter to Vehicle) IR 리모컨 시스템.

ESP32-S3 Super Mini 두 대를 이용하여 NEC IR 신호로 차량에 START/STOP 명령을 전달하고,
ROS1 토픽으로 변환하여 자율주행 레이싱 스택과 연동한다.

## 구성

```
t2v_transmitter/   # ESP32-S3 IR 송신기 (NEC 38kHz + BLE GATT)
t2v_receiver/      # ESP32-S3 IR 수신기 (NEC 수신 + USB CDC + BLE GATT)
t2v_node/          # ROS1 패키지 (USB CDC → /joy 토픽 브릿지)
```

## 동작 흐름

```
송신기 키(1/2/3) → NEC IR 38kHz → 수신기 VS1838B
                                      ↓
                              NEC 디코딩 + USB CDC
                                      ↓
                              t2v_bridge.py (ROS1)
                                      ↓
                              /joy (sensor_msgs/Joy)
                                      ↓
                              simple_mux.py (레이싱 스택)
                                      ↓
                              자율주행 시작/정지
```

## 명령어

| 키 | IR 코드 | Joy 매핑 | 동작 |
|----|---------|----------|------|
| 1 | START_GO (0x02) | buttons[5] (RB) | 자율주행 시작 |
| 2 | STOP (0x7F) | buttons[4] (LB) + axes[1]=0 | 정지 |
| 3 | START_ABORT (0x03) | buttons[4] (LB) + axes[1]=0 | 긴급 정지 |

## 빌드

### ESP32 펌웨어 (ESP-IDF v5.3)

```bash
# 송신기
cd t2v_transmitter
idf.py build
idf.py -p /dev/ttyACM1 flash monitor

# 수신기
cd t2v_receiver
idf.py build
# BOOT+RST로 다운로드 모드 진입 후
idf.py -p /dev/ttyACM0 flash
```

### ROS1 노드 (Noetic)

```bash
# catkin 워크스페이스에 t2v_node 복사 후
catkin_make
rosrun t2v_node t2v_bridge.py
```

## 하드웨어

| 항목 | 송신기 | 수신기 |
|------|--------|--------|
| 보드 | ESP32-S3 Super Mini | ESP32-S3 Super Mini |
| IR | GPIO5 (IR LED) | GPIO6 (VS1838B) |
| USB | USB Serial/JTAG | TinyUSB CDC (VID:0x5455 PID:0x1911) |
| BLE | T2V-Transmitter | T2V-Receiver |

## 문서

- [T2V_CHECKPOINT.md](T2V_CHECKPOINT.md) - 프로젝트 상세 스펙
- [T2V_INTEGRATION.md](T2V_INTEGRATION.md) - 레이싱 스택 통합 가이드

## 팀

UNICORN Racing - ICRA 2026 RoboRacer
