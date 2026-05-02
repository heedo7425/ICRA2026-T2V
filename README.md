# ICRA2026-T2V

ICRA 2026 RoboRacer 대회용 **IR 리모컨 시스템**입니다.
ESP32-S3 보드 두 개로 만든 송신기/수신기로, 38kHz NEC IR 신호를 통해 차량에 START / STOP 명령을 전달합니다.

처음 보는 사람도 따라 할 수 있도록 작성했습니다. 순서대로 따라가면 됩니다.

---

## 1. 시스템 개요

```
   [송신기]                                    [수신기]
   ESP32-S3 Super Mini                         ESP32-S3 Super Mini
   + IR LED (GPIO5)         ──IR 38kHz──▶      + VS1838B IR 수신모듈 (GPIO6)
   키보드 1/2/3 입력                            USB CDC 4바이트 출력
                                                BLE Notify (FF03)
```

| 키 | 명령 | 코드 |
|----|------|------|
| `1` | START_GO | `0x02` |
| `2` | STOP | `0x7F` |
| `3` | START_ABORT | `0x03` |

> 이 README는 **ESP32 펌웨어(송신기 / 수신기)** 의 빌드·플래시·동작 확인까지만 다룹니다.
> ROS 연동(`t2v_node/`)은 다른 문서에서 다룹니다.

---

## 2. 준비물

### 하드웨어
- ESP32-S3 Super Mini × **2개** (송신기, 수신기)
- IR LED (940nm) + 직렬저항 (예: 100Ω) 또는 IR LED 드라이버 회로
- VS1838B (또는 호환 38kHz IR 수신모듈) × 1개
- USB-C 케이블 × 2개
- 점퍼 와이어, 브레드보드

### 소프트웨어
- **Ubuntu 20.04 / 22.04** (Linux 환경 권장)
- **ESP-IDF v5.3.x**
- Python 3.8+, `git`

---

## 3. ESP-IDF 설치 (한 번만 하면 됨)

이미 설치되어 있다면 4번으로 넘어가세요.

```bash
# 1) 의존 패키지
sudo apt update && sudo apt install -y git wget flex bison gperf python3 python3-pip \
    python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# 2) ESP-IDF v5.3.2 클론
mkdir -p ~/esp && cd ~/esp
git clone --recursive --branch v5.3.2 https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3

# 3) 환경 변수 적용 (새 터미널 열 때마다 실행 필요)
. ~/esp/esp-idf/export.sh
```

> 매번 치기 귀찮다면 `~/.bashrc`에 다음 줄을 추가:
> ```bash
> alias get_idf='. $HOME/esp/esp-idf/export.sh'
> ```

### 시리얼 권한
플래시 시 `Permission denied: /dev/ttyACM0` 오류가 나면:
```bash
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인
```

---

## 4. 코드 받기

```bash
git clone https://github.com/heedo7425/ICRA2026-T2V.git
cd ICRA2026-T2V
```

디렉토리 구조:
```
t2v_transmitter/   # 송신기 펌웨어
t2v_receiver/      # 수신기 펌웨어
t2v_node/          # (ROS 패키지 — 이 README에서는 다루지 않음)
```

---

## 5. 하드웨어 연결

### 송신기 (t2v_transmitter)

| ESP32-S3 핀 | 연결 |
|---|---|
| **GPIO5** | IR LED 애노드 → 저항 → GND (또는 드라이버 회로 입력) |
| 5V / GND | USB로 자동 공급 |

### 수신기 (t2v_receiver) — VS1838B 결선

VS1838B는 정면(돔이 보이는 쪽)을 봤을 때 핀이 왼쪽부터 **OUT / GND / VCC** 입니다.

| VS1838B 핀 | ESP32-S3 핀 |
|---|---|
| OUT | **GPIO6** |
| GND | GND |
| VCC | 3.3V |

> VS1838B 출력은 Active LOW (수신 시 0V). 펌웨어에서 `invert_in = true` 로 보정합니다.

---

## 6. 빌드 & 플래시

송신기와 수신기 각각에 대해 같은 절차를 반복합니다.
먼저 새 터미널에서 ESP-IDF 환경을 활성화:
```bash
. ~/esp/esp-idf/export.sh
```

### 6-1. 송신기 플래시

```bash
cd t2v_transmitter

# 처음 한 번만: 타깃 설정
idf.py set-target esp32s3

# 빌드
idf.py build

# 보드를 USB로 연결한 뒤 플래시 + 모니터
idf.py -p /dev/ttyACM1 flash monitor
```

부팅이 정상이면 모니터에 다음과 비슷한 로그가 뜹니다:
```
I (xxx) T2V_TX: NimBLE host started, advertising as 'T2V-Transmitter'
I (xxx) T2V_TX: Keyboard task started. Press 1/2/3.
```

> 모니터 종료: `Ctrl+]`

### 6-2. 수신기 플래시

```bash
cd ../t2v_receiver

# 처음 한 번만
idf.py set-target esp32s3

# 빌드 (TinyUSB 의존성 자동 다운로드)
idf.py build

# 플래시 (수신기는 USB CDC를 펌웨어가 점유하므로 monitor 는 함께 띄우지 않음)
idf.py -p /dev/ttyACM0 flash
```

#### 다운로드 모드 진입이 필요할 때
ESP32-S3가 자동으로 다운로드 모드에 못 들어가면 수동으로:
1. **BOOT 버튼**을 누른 채로
2. **RST 버튼**을 눌렀다 떼고
3. **BOOT 버튼**을 떼면 다운로드 모드 진입.
4. `idf.py flash` 명령 다시 실행.

#### 포트 번호가 다를 때
`ls /dev/ttyACM*` 또는 `ls /dev/ttyUSB*`로 확인 후 명령의 `-p` 인자를 맞춥니다.
보드가 두 개 꽂혀 있을 때는 한 번에 하나씩만 연결해서 헷갈리지 않게 합니다.

---

## 7. 동작 확인

송신기와 수신기 모두 플래시한 뒤, 두 보드를 모두 USB로 연결합니다.
IR LED가 VS1838B 정면을 향하게 배치 (수십 cm 이내).

### 7-1. 수신기 USB CDC 출력 모니터링

새 터미널에서:
```bash
sudo apt install -y python3-serial
python3 -c "
import serial
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
print('Listening on /dev/ttyACM0...')
while True:
    data = s.read(4)
    if len(data) == 4:
        print('RX:', ' '.join(f'{b:02X}' for b in data))
"
```

### 7-2. 송신기에서 키 입력

송신기는 USB Serial/JTAG로 키를 받습니다. 새 터미널에서:
```bash
. ~/esp/esp-idf/export.sh
cd t2v_transmitter
idf.py -p /dev/ttyACM1 monitor
```
모니터가 뜨면 키보드에서 `1`, `2`, `3` 을 누릅니다 (모니터 창에 포커스가 있어야 함).

### 7-3. 기대 결과

| 송신기 키 | 수신기 측 USB CDC 출력 (4바이트) |
|---|---|
| `1` | `FF 00 02 FD` |
| `2` | `FF 00 7F 80` |
| `3` | `FF 00 03 FC` |

형식: `[addr][~addr][cmd][~cmd]`. 4바이트가 안 나오면 IR LED 방향, 거리, GPIO 결선을 다시 확인.

---

## 8. BLE 동작 확인 (선택)

스마트폰에 **nRF Connect** 앱을 설치한 뒤 스캔하면 다음 두 디바이스가 보입니다.

### 송신기 (`T2V-Transmitter`)
- Service `0x00FF`
  - Char `0xFF01` (target_address): Read / Write — 1바이트 (예: `0xFF` = 브로드캐스트)

### 수신기 (`T2V-Receiver`)
- Service `c902d400-1809-2a94-904d-af5cbdcefe9b`

  | UUID | 이름 | 속성 |
  |---|---|---|
  | `0xFF01` | Team Name | Read |
  | `0xFF02` | Car Name | Read |
  | `0xFF03` | IR NEC frames | Read + **Notify** |
  | `0xFF04` | Unicast addr | Read + Write |
  | `0xFF05` | Multicast mask | Read + Write |

`0xFF03`에 Notify를 켠 뒤 송신기 키를 누르면 4바이트가 푸시됩니다.

---

## 9. 자주 막히는 곳

| 증상 | 원인 / 해결 |
|---|---|
| `idf.py: command not found` | `. ~/esp/esp-idf/export.sh` 안 함 |
| `Permission denied: /dev/ttyACM0` | `sudo usermod -aG dialout $USER` 후 재로그인 |
| 플래시 중 `Failed to connect` | BOOT+RST로 다운로드 모드 수동 진입 |
| 빌드 시 `tinyusb.h: No such file` | 수신기 폴더에서 `idf.py build` 다시 실행 (의존성 자동 다운로드) |
| 송신기 키 입력 무반응 | 모니터 터미널에 포커스 두고 키 입력 (`getchar()` 사용 중) |
| 수신기 USB CDC 4바이트가 안 나옴 | IR LED 방향 / 거리(<50cm) / VS1838B Vcc(3.3V) 확인 |
| 수신은 되는데 IR 코드가 깨짐 | IR LED 직렬저항이 너무 크면 출력 약함. 100Ω 부근으로 |

---

## 10. 참고 문서

- [T2V_CHECKPOINT.md](T2V_CHECKPOINT.md) — 펌웨어 내부 구조 / sdkconfig 상세
- [T2V_INTEGRATION.md](T2V_INTEGRATION.md) — ROS1 racing stack 연동 가이드

---

UNICORN Racing — ICRA 2026 RoboRacer
