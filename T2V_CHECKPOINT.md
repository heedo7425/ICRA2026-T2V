# T2V 프로젝트 중간 점검 (2026-04-12)

## 개요

ESP32-S3 Super Mini 두 대를 이용한 IR 리모컨 시스템.
송신기(Transmitter)가 NEC IR 신호를 쏘면 수신기(Receiver)가 받아 처리하고,
BLE GATT 서버로 외부 설정을 받으며, USB CDC로 PC에 데이터를 보낸다.

---

## 하드웨어 구성

| 항목 | 송신기 | 수신기 |
|------|--------|--------|
| 보드 | ESP32-S3 Super Mini | ESP32-S3 Super Mini |
| IR 핀 | GPIO5 (IR LED 드라이버 입력) | GPIO6 (VS1838B 출력, Active LOW) |
| 콘솔 포트 | `/dev/ttyACM1` (USB Serial/JTAG) | `/dev/ttyACM0` (UART0, TX: IO43) |
| 빌드 스크립트 | `./idf.sh` → `ESPPORT=/dev/ttyACM1` | `./idf.sh` → `ESPPORT=/dev/ttyACM0` |

---

## 프로젝트 디렉토리

```
~/esp_ws/
├── esp-idf/                  # ESP-IDF v5.3.2
├── t2v_transmitter/
│   ├── main/t2v_transmitter.c
│   ├── sdkconfig.defaults
│   └── idf.sh                # ESPPORT=/dev/ttyACM1
└── t2v_receiver/
    ├── main/
    │   ├── t2v_receiver.c
    │   └── idf_component.yml  # espressif/esp_tinyusb: "*"
    ├── managed_components/
    │   ├── espressif__esp_tinyusb/  (v2.1.1)
    │   └── espressif__tinyusb/     (v0.19.0~2)
    ├── sdkconfig.defaults
    └── idf.sh                # ESPPORT=/dev/ttyACM0
```

---

## 송신기 (t2v_transmitter)

### 구현 완료 기능

- **NEC IR 송신** (GPIO5, 38kHz 캐리어)
  - RMT TX 채널 + Copy Encoder 사용
  - `signal_range_min_ns = 1000` (1µs), `signal_range_max_ns = 14000000` (14ms)
  - 프레임 구조: 9ms 리더 + 4.5ms 스페이스 + 32비트 데이터 (LSB first) + 종료 펄스

- **키보드 입력** (USB Serial/JTAG via `getchar()`)
  - `1` → `START_GO (0x02)` 송신
  - `2` → `STOP (0x7F)` 송신
  - `3` → `START_ABORT (0x03)` 송신
  - `uart_driver_install()` 미사용 (충돌 방지), `getchar()`만 사용

- **BLE GATT 서버** (NimBLE)
  - 디바이스명: `T2V-Transmitter`
  - Service UUID: `0x00FF`
  - Characteristic `0xFF01` (`target_address`): Read+Write, 초기값 `0x00`
  - BLE Write로 목표 주소 변경 → 다음 IR 송신에 즉시 반영

### sdkconfig.defaults 주요 설정

```
CONFIG_ESP_CONSOLE_UART_NONE=y
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=n
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1
```

### 태스크 구조

| 태스크 | 기능 |
|--------|------|
| `nimble_host_task` | NimBLE BLE 이벤트 루프 |
| `keyboard_task` | `getchar()` 키 입력 → IR 송신 |

---

## 수신기 (t2v_receiver)

### 구현 완료 기능

- **NEC IR 수신** (GPIO6, VS1838B)
  - RMT RX 채널, `invert_in = true` (Active LOW 보정)
  - `signal_range_min_ns = 1000`, `signal_range_max_ns = 14000000`
  - ISR 콜백 → FreeRTOS Queue → 태스크에서 NEC 디코딩
  - 디코딩: 리더 펄스 검증 + 32비트 LSB-first + 주소/명령 반전 검증

- **USB CDC** (TinyUSB, esp_tinyusb v2.1.1)
  - VID: `0x5455`, PID: `0x1911`
  - NEC 디코딩 성공 시 4바이트 전송: `[addr][~addr][cmd][~cmd]`
  - DTR 기반 연결/해제 감지 (`cdc_line_state_changed_cb`)
  - 디버그 로그: UART0 (TX: IO43, 115200bps)
  - TinyUSB v2 API: `tinyusb_config_t.descriptor.device`, `tinyusb_cdcacm_init()`

- **BLE GATT 서버** (NimBLE, 공식 T2V 스펙)
  - 디바이스명: `T2V-Receiver`
  - Service UUID: `c902d400-1809-2a94-904d-af5cbdcefe9b` (128-bit)
  - NimBLE little-endian: `BLE_UUID128_INIT(0x9b,0xfe,0xce,0xbd,0x5c,0xaf,0x4d,0x90,0x94,0x2a,0x09,0x18,0x00,0xd4,0x02,0xc9)`

  | UUID | 이름 | 속성 | 초기값 |
  |------|------|------|--------|
  | `0xFF01` | Team Name | Read | `"Unicorn Racing"` |
  | `0xFF02` | Car Name | Read | `"Unicorn01"` |
  | `0xFF03` | IR NEC frames | Read + Notify | `[addr,~addr,cmd,~cmd]` |
  | `0xFF04` | Unicast addr | Read + Write | `0x01` |
  | `0xFF05` | Multicast mask | Read + Write | `0x01` |

- **주소 필터링**
  - 브로드캐스트 `0xFF` → 무조건 처리
  - 유니캐스트 `== FF04` → 처리
  - 멀티캐스트 `0x00~0x07` → `(FF05 >> addr) & 1` 비트 확인
  - USB 전송 및 BLE Notify는 필터 무관, 항상 실행

### NEC 수신 후 처리 순서

```
RMT ISR → Queue → ir_rx_task
    ├─ ① USB CDC 4바이트 전송   (필터 무관)
    ├─ ② BLE FF03 Notify        (필터 무관)
    └─ ③ addr_filter() 통과 시 → t2v_process_command()
```

### sdkconfig.defaults 주요 설정

```
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=n
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_RX_BUFSIZE=64
CONFIG_TINYUSB_CDC_TX_BUFSIZE=64
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200
```

### idf_component.yml

```yaml
dependencies:
  espressif/esp_tinyusb: "*"
```

### 태스크 구조

| 태스크 | 기능 |
|--------|------|
| `nimble_host_task` | NimBLE BLE 이벤트 루프 |
| `ir_rx_task` | RMT 수신 → NEC 디코딩 → USB + BLE + 명령 처리 |

---

## T2V 명령어 코드

| 명령 | 코드 | 설명 |
|------|------|------|
| `START_GO` | `0x02` | 출발 신호 |
| `START_ABORT` | `0x03` | 출발 취소 |
| `STOP` | `0x7F` | 정지 신호 |
| 브로드캐스트 주소 | `0xFF` | 전체 대상 |

---

## 빌드 상태

| 프로젝트 | 빌드 | 플래시 |
|----------|------|--------|
| `t2v_transmitter` | 완료 | 미확인 (이전 세션) |
| `t2v_receiver` | **완료** (2026-04-12) | 미실행 |

### 플래시 명령

```bash
# 수신기 플래시 + 모니터
cd ~/esp_ws/t2v_receiver
./idf.sh flash monitor

# 송신기 플래시 + 모니터
cd ~/esp_ws/t2v_transmitter
./idf.sh flash monitor
```

> 수신기 모니터는 UART0 포트(`/dev/ttyACM0`, 115200bps)로 출력된다.
> 송신기 모니터는 USB Serial/JTAG(`/dev/ttyACM1`)으로 출력된다.

---

## 주요 트러블슈팅 이력

| 문제 | 원인 | 해결 |
|------|------|------|
| `signal_range_min_ns too big` | `RMT_MIN_NS = 100000` (100µs) | `1000` (1µs)으로 수정 |
| 키보드 입력 무응답 | `uart_driver_install()` + USB Serial/JTAG 충돌 | UART 드라이버 제거, `getchar()` 사용 |
| `tinyusb.h: No such file` | ESP-IDF 5.3에서 TinyUSB 내장 제거 | `idf.py add-dependency espressif/esp_tinyusb` |
| TinyUSB 컴파일 에러 | v2 API 구조체 멤버명 변경 | `descriptor.device`, `tinyusb_cdcacm_init()` 등으로 수정 |

---

## 남은 작업

- [ ] 수신기 실보드 플래시 및 동작 확인
- [ ] 송신기-수신기 통신 테스트 (IR 수신 → USB CDC 출력 확인)
- [ ] BLE 연결 후 특성 Read/Write 테스트 (nRF Connect 등)
- [ ] PC에서 USB CDC `/dev/ttyACM0` 수신 확인 (`cat /dev/ttyACM0` 또는 Python pyserial)
