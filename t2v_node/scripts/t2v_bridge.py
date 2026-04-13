#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T2V USB CDC → ROS1 브릿지 노드

ESP32-S3 수신기의 USB CDC 포트에서 4바이트 NEC 프레임을 읽고
ROS1 토픽으로 퍼블리시한다.

NEC 프레임 형식: [주소][~주소][명령어][~명령어]
- 주소 + ~주소 = 0xFF (무결성 검증)
- 명령어 + ~명령어 = 0xFF (무결성 검증)

퍼블리시 토픽:
  /t2v/command    (t2v_node/T2VCommand) — 원시 T2V 명령어
  /joy            (sensor_msgs/Joy)     — 레이싱 스택 호환 (simple_mux.py)
"""

import os
import glob
import rospy
import serial
from sensor_msgs.msg import Joy
from t2v_node.msg import T2VCommand

# ── T2V 명령어 코드 → 이름 매핑 ──
CMD_MAP = {
    0x02: "START_GO",
    0x03: "START_ABORT",
    0x7F: "STOP",
}

# ── T2V 수신기 USB CDC 식별자 ──
T2V_VID = "5455"
T2V_PID = "1911"


def find_t2v_port():
    """
    USB CDC 포트 자동 감지.
    /sys/class/tty 에서 VID:PID가 5455:1911 인 디바이스를 찾는다.
    못 찾으면 None 반환.
    """
    for tty_path in glob.glob("/sys/class/tty/ttyACM*"):
        device_path = os.path.join(tty_path, "device")
        if not os.path.islink(device_path):
            continue

        # USB 디바이스 경로를 따라 올라가며 idVendor/idProduct 확인
        usb_path = os.path.realpath(device_path)
        while usb_path != "/":
            vid_file = os.path.join(usb_path, "idVendor")
            pid_file = os.path.join(usb_path, "idProduct")
            if os.path.isfile(vid_file) and os.path.isfile(pid_file):
                with open(vid_file) as f:
                    vid = f.read().strip()
                with open(pid_file) as f:
                    pid = f.read().strip()
                if vid == T2V_VID and pid == T2V_PID:
                    dev_name = os.path.basename(tty_path)
                    return "/dev/{}".format(dev_name)
            usb_path = os.path.dirname(usb_path)

    return None


def main():
    rospy.init_node("t2v_node", anonymous=False)

    # ── ROS 파라미터 ──
    param_port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)

    # ── 포트 자동 감지 시도 ──
    detected = find_t2v_port()
    if detected:
        port = detected
        rospy.loginfo("T2V 수신기 자동 감지: %s (VID:%s PID:%s)", port, T2V_VID, T2V_PID)
    else:
        port = param_port
        rospy.logwarn("T2V 수신기 자동 감지 실패 → 파라미터 포트 사용: %s", port)

    # ── 퍼블리셔 ──
    pub_cmd = rospy.Publisher("/t2v/command", T2VCommand, queue_size=10)
    pub_joy = rospy.Publisher("/joy", Joy, queue_size=10)

    # ── 시리얼 연결 루프 ──
    # 연결 실패 시 재시도, Ctrl+C로 종료
    while not rospy.is_shutdown():
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            rospy.loginfo("포트 연결 성공: %s @ %d baud", port, baud)
        except serial.SerialException as e:
            rospy.logwarn("포트 열기 실패 (%s): %s — 3초 후 재시도", port, e)
            rospy.sleep(3.0)
            continue

        # ── 수신 루프 ──
        # 프레임 형식: [0x55][0xAA][addr][~addr][cmd][~cmd]
        # 싱크 헤더(0x55 0xAA)를 찾은 뒤 4바이트 NEC 데이터를 읽는다.
        try:
            ser.reset_input_buffer()
            sync_state = 0  # 0: 0x55 대기, 1: 0xAA 대기

            while not rospy.is_shutdown():
                # ── 싱크 헤더 탐색 ──
                if sync_state == 0:
                    byte = ser.read(1)
                    if len(byte) == 0:
                        continue
                    if byte[0] == 0x55:
                        sync_state = 1
                    continue

                if sync_state == 1:
                    byte = ser.read(1)
                    if len(byte) == 0:
                        sync_state = 0
                        continue
                    if byte[0] == 0xAA:
                        sync_state = 2  # 싱크 완료, 데이터 읽기
                    elif byte[0] == 0x55:
                        sync_state = 1  # 0x55가 연속으로 올 수 있음
                    else:
                        sync_state = 0
                    continue

                # ── 싱크 완료: 4바이트 NEC 데이터 읽기 ──
                data = ser.read(4)
                sync_state = 0  # 다음 프레임을 위해 리셋

                if len(data) < 4:
                    continue

                addr, addr_inv, cmd, cmd_inv = data

                # ── 무결성 검증 ──
                if (addr + addr_inv) & 0xFF != 0xFF or \
                   (cmd + cmd_inv) & 0xFF != 0xFF:
                    rospy.logwarn(
                        "무결성 실패: [0x%02X 0x%02X 0x%02X 0x%02X]",
                        addr, addr_inv, cmd, cmd_inv
                    )
                    continue

                # ── 명령어 이름 변환 ──
                cmd_name = CMD_MAP.get(cmd, "UNKNOWN")

                rospy.loginfo("수신: %s (주소: 0x%02X)", cmd_name, addr)

                # ── /t2v/command 퍼블리시 ──
                msg = T2VCommand()
                msg.address = addr
                msg.command = cmd
                msg.command_name = cmd_name
                pub_cmd.publish(msg)

                # ── /joy 퍼블리시 (sensor_msgs/Joy, Xbox 호환) ──
                # simple_mux.py가 buttons[5](RB), buttons[4](LB)로 모드 전환
                joy_msg = Joy()
                joy_msg.header.stamp = rospy.Time.now()
                joy_msg.axes = [0.0] * 8       # Xbox 표준 8축
                joy_msg.buttons = [0] * 11     # Xbox 표준 11버튼

                if cmd_name == "START_GO":
                    # RB → 자율주행 모드 진입
                    joy_msg.buttons[5] = 1
                    pub_joy.publish(joy_msg)

                elif cmd_name in ("STOP", "START_ABORT"):
                    # LB + 속도 0 → 수동 모드 정지
                    # 확실한 정지를 위해 0.5초간 반복 전송 (50Hz)
                    joy_msg.buttons[4] = 1
                    joy_msg.axes[1] = 0.0       # 속도 = 0
                    rate = rospy.Rate(50)
                    for _ in range(25):         # 0.5초 × 50Hz = 25회
                        if rospy.is_shutdown():
                            break
                        joy_msg.header.stamp = rospy.Time.now()
                        pub_joy.publish(joy_msg)
                        rate.sleep()

        except serial.SerialException as e:
            rospy.logwarn("포트 연결 해제: %s — 3초 후 재연결 시도", e)
            ser.close()
            rospy.sleep(3.0)
        except rospy.ROSInterruptException:
            break

    # ── 종료 ──
    if "ser" in dir() and ser.is_open:
        ser.close()
    rospy.loginfo("T2V 노드 종료")


if __name__ == "__main__":
    main()
