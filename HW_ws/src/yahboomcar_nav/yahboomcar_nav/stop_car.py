#!/usr/bin/env python                          # 실행 시 파이썬 인터프리터를 지정하는 shebang (ROS2에선 python3 권장)
# coding:utf-8                                  # 소스 파일의 문자 인코딩을 UTF-8로 지정
import rclpy                                    # ROS 2 Python 클라이언트 라이브러리 임포트
from rclpy.node import Node                     # ROS 2 노드의 기본 클래스 임포트
from sensor_msgs.msg import LaserScan           # (미사용) LaserScan 메시지 타입 임포트
import numpy as np                              # (미사용) 수치 계산 라이브러리 NumPy 임포트
from rclpy.clock import Clock                   # (미사용) ROS 2 Clock 인터페이스 임포트
import os                                       # 셸 명령 실행을 위해 OS 모듈 임포트
import sys                                      # (미사용) 파이썬 시스템 관련 유틸 임포트
from geometry_msgs.msg import Twist             # (미사용) 속도 명령 메시지 타입 임포트 (실제 코드는 shell로 발행)

class Stop_Car(Node):                           # Stop_Car라는 이름의 ROS 2 노드 클래스 정의
    def __init__(self,name):                    # 생성자: 외부에서 노드 이름을 전달받음
        super().__init__(name)                  # 부모(Node) 생성자 호출로 노드 초기화
        
    def exit_pro(self):                         # 종료 시 호출할 사용자 정의 메서드
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "  # ROS2 CLI로 /cmd_vel에 1회 퍼블리시하는 명령 일부
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''  # Twist 메시지 본문(선속/각속 모두 0)
        cmd = cmd1 +cmd2                         # 두 문자열을 이어서 하나의 셸 명령으로 만듦
        os.system(cmd)                           # OS 셸을 통해 명령 실행(외부 프로세스 생성 방식)

def main():                                      # 파이썬 스크립트 진입점 함수
    rclpy.init()                                 # rclpy 초기화(ROS 2 통신 준비)
    stop_car = Stop_Car("StopCarNode")           # Stop_Car 노드 인스턴스 생성(노드명: StopCarNode)
    try:                                         # 예외 처리 블록 시작
        rclpy.spin(stop_car)                     # 콜백 처리 루프(여기서는 콜백 없음 → Ctrl+C 대기)
    except KeyboardInterrupt:                    # 키보드 인터럽트(Ctrl+C) 발생 시
        pass                                     # 특별한 처리 없이 통과
    finally:                                     # 위 블록 종료 시 항상 실행되는 영역
        stop_car.exit_pro()                      # 종료 직전에 0 속도 Twist를 1회 퍼블리시(셸 명령 방식)
        stop_car.destroy_node()                  # 노드 자원 정리
        rclpy.shutdown()                         # rclpy 종료(ROS 2 정리)
