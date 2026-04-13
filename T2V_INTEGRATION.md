# T2V IR 리모컨 → ICRA2026_HJ 레이싱 스택 통합 가이드

## 개요

T2V IR 리모컨 시스템의 ROS 노드(`t2v_bridge.py`)가 퍼블리시하는 신호를
ICRA2026_HJ 자율주행 레이싱 스택의 조이스틱 제어 체계에 연결하는 방법.

---

## 현재 시스템 구조

### T2V 측 (우리 시스템)

```
송신기 키(1/2/3) → IR → 수신기 → USB CDC → t2v_bridge.py
  ├─ /t2v/command    (t2v_node/T2VCommand)
  └─ /joy/joy_node   (std_msgs/String)
       ├─ "rb"           ← START_GO (0x02)
       ├─ "lb"           ← STOP (0x7F)
       └─ "START_ABORT"  ← START_ABORT (0x03)
```

### 레이싱 스택 측 (ICRA2026_HJ)

```
Xbox 컨트롤러 → joy_node → /joy (sensor_msgs/Joy)
                              ↓
                        simple_mux.py
                              ↓
              ┌───────────────┴───────────────┐
              │                               │
     buttons[5]=1 (RB)              buttons[4]=1 (LB)
     → current_host = "autodrive"   → current_host = "humandrive"
     → 자율주행 명령 전달              → 조이스틱 수동 조종
              │                               │
              ↓                               ↓
    /low_level/.../output          /low_level/.../output
              ↓                               ↓
         VESC 드라이버 → 모터/서보 제어
```

### 핵심 파일

| 파일 | 경로 | 역할 |
|------|------|------|
| simple_mux.py | stack_master/scripts/simple_mux.py | 조이스틱/자율주행 명령 멀티플렉서 |
| joy_teleop.yaml | stack_master/config/\<Car\>/devices/joy_teleop.yaml | 버튼/축 매핑 설정 |
| vesc.yaml | stack_master/config/\<Car\>/devices/vesc.yaml | VESC 모터/서보 + 조이스틱 파라미터 |
| low_level.launch | stack_master/launch/low_level.launch | joy_node 등 하드웨어 시작 |
| state_machine_node.py | state_machine/src/state_machine_node.py | 행동 상태 머신 |

---

## 문제점

T2V는 `std_msgs/String`으로 `"rb"`, `"lb"`를 퍼블리시하지만,
레이싱 스택의 `simple_mux.py`는 `sensor_msgs/Joy`를 구독한다.

**메시지 형식이 다르다.**

```
T2V 출력:  std_msgs/String ("rb")
스택 입력: sensor_msgs/Joy (buttons[5]=1)
```

---

## 통합 방법

### 방법 1: t2v_bridge.py에서 직접 Joy 메시지 퍼블리시 (권장)

t2v_bridge.py를 수정하여 `/joy/joy_node` (String) 대신
`/joy` (sensor_msgs/Joy)를 직접 퍼블리시한다.

#### 매핑

| T2V 명령 | IR 코드 | Joy 매핑 | 동작 |
|----------|---------|----------|------|
| START_GO | 0x02 | buttons[5]=1 (RB) | 자율주행 모드 진입 |
| STOP | 0x7F | buttons[4]=1 (LB) + axes[1]=0.0 | 수동 모드 + 정지 |
| START_ABORT | 0x03 | buttons[4]=1 (LB) + axes[1]=0.0 | 수동 모드 + 정지 |

#### 코드 변경 (t2v_bridge.py)

```python
from sensor_msgs.msg import Joy

# 퍼블리셔 변경
pub_joy = rospy.Publisher("/joy", Joy, queue_size=10)

# Joy 메시지 생성 함수
def make_joy_msg(buttons_map):
    """Xbox 컨트롤러 호환 Joy 메시지 생성"""
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.axes = [0.0] * 8      # 8축 (Xbox 표준)
    msg.buttons = [0] * 11    # 11버튼 (Xbox 표준)
    for idx, val in buttons_map.items():
        msg.buttons[idx] = val
    return msg

# 퍼블리시 부분 변경
if cmd_name == "START_GO":
    # RB(buttons[5]) → 자율주행 모드
    pub_joy.publish(make_joy_msg({5: 1}))

elif cmd_name == "STOP":
    # LB(buttons[4]) + 속도 0 → 수동 모드 정지
    msg = make_joy_msg({4: 1})
    msg.axes[1] = 0.0  # 속도 = 0
    pub_joy.publish(msg)

elif cmd_name == "START_ABORT":
    # LB(buttons[4]) + 속도 0 → 수동 모드 정지
    msg = make_joy_msg({4: 1})
    msg.axes[1] = 0.0
    pub_joy.publish(msg)
```

#### 주의사항

- `simple_mux.py`의 freshness threshold이 **1.0초**이므로,
  Joy 메시지가 1초 이내에 도착해야 유효하게 처리된다.
- STOP/START_ABORT 시 Joy 메시지를 **여러 번 반복 전송**(0.5초간 50Hz)해야
  확실하게 수동 모드로 전환되고 차가 멈춘다.
- START_GO는 1회 전송으로 충분 (상태 전환만 하면 됨).

### 방법 2: 별도 브릿지 노드 추가

t2v_bridge.py는 그대로 두고, 별도 노드가
`/joy/joy_node` (String) → `/joy` (Joy)로 변환한다.

```
t2v_bridge.py → /joy/joy_node (String) → t2v_joy_bridge.py → /joy (Joy)
```

장점: 기존 코드 수정 최소화
단점: 노드 하나 더 실행해야 함

---

## 레이싱 스택에서 변경할 것

### 1. low_level.launch에서 joy_node 제거 또는 유지

- **Xbox 컨트롤러도 같이 쓸 경우**: joy_node 유지. T2V와 Xbox 둘 다 `/joy`에 퍼블리시.
- **T2V만 쓸 경우**: joy_node 비활성화 가능 (launch 파일에서 주석 처리).

### 2. simple_mux.py 수동 정지 강화 (선택)

현재 LB는 "수동 조종 모드"인데 축 입력(axes[1])이 0이면 정지.
T2V에서 STOP 시 axes[1]=0.0을 보내므로 정지는 동작하지만,
**확실한 긴급 정지**가 필요하면 simple_mux.py에 로직 추가 가능:

```python
# simple_mux.py의 joy_callback에 추가
if joy_msg.buttons[4] == 1 and joy_msg.axes[1] == 0.0:
    # 강제 정지: 즉시 속도 0 명령
    stop_msg = AckermannDriveStamped()
    stop_msg.drive.speed = 0.0
    stop_msg.drive.steering_angle = 0.0
    self.pub_output.publish(stop_msg)
```

---

## 실행 순서

### 레이싱 현장에서

```bash
# 터미널 1: 상태 추정 (로봇 PC)
roslaunch glim_ros glil_cpu.launch

# 터미널 2: 3D 레이싱 시스템 (로봇 PC)
roslaunch stack_master car_race.launch map:=<맵이름> racecar_version:=SRX1

# 터미널 3: 컨트롤러 + 상태 머신 (로봇 PC)
roslaunch stack_master headtohead.launch

# 터미널 4: T2V 브릿지 (Docker 컨테이너)
docker exec -it t2v-ros bash
rosrun t2v_node t2v_bridge.py
```

### 동작 흐름

```
1. 시스템 시작 → 차량 정지 상태 (수동 모드)
2. IR 리모컨 '1' (START_GO) → RB 신호 → 자율주행 시작
3. 자율주행 중...
4. IR 리모컨 '2' (STOP) → LB + 속도 0 → 수동 모드 정지
5. IR 리모컨 '3' (START_ABORT) → LB + 속도 0 → 수동 모드 정지
6. 다시 '1' → 자율주행 재개
```

---

## 토픽 연결도 (통합 후)

```
[T2V 송신기]                    [T2V 수신기]
  키 1/2/3                    ESP32-S3 + VS1838B
     │                              │
     │ NEC IR 38kHz                 │ USB CDC (0x55 0xAA + 4byte)
     ↓                              ↓
                              [t2v_bridge.py]
                                    │
                        ┌───────────┼───────────┐
                        ↓           ↓           ↓
                /t2v/command    /joy (Joy)   로그 출력
                (T2VCommand)   buttons[4/5]
                                    │
                                    ↓
                            [simple_mux.py]
                                    │
                    ┌───────────────┴───────────────┐
                    ↓                               ↓
              autodrive (RB)                  humandrive (LB)
              자율주행 명령 전달               속도=0 정지
                    ↓                               ↓
              [controller_manager.py]         즉시 정지 명령
                    ↓                               │
              L1/MAP 컨트롤러                       │
                    ↓                               ↓
              /low_level/.../output ←───────────────┘
                    ↓
              [VESC 드라이버]
                    ↓
              모터 + 서보
```

---

## 체크리스트

- [ ] t2v_bridge.py에 Joy 메시지 퍼블리시 추가 (방법 1)
- [ ] STOP/START_ABORT 시 반복 전송 로직 구현
- [ ] 레이싱 스택과 같은 ROS 네트워크에 연결 (ROS_MASTER_URI 설정)
- [ ] simple_mux.py가 T2V의 Joy 메시지를 수신하는지 확인
- [ ] 실차 테스트: START_GO → 자율주행, STOP → 정지
- [ ] 긴급 정지 동작 확인 (1초 이내 반응)
