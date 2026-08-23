#!/usr/bin/env python3
import subprocess

def call_kuka_initial_pose():
    """KUKA 로봇의 초기 위치로 이동하는 서비스 호출"""
    try:
        # ros2 service call 명령어 실행
        subprocess.run([
            'ros2', 'service', 'call', '/kuka_joint',
            'kuka_control_box_srvs/srv/KukaJoint',
            '{a1: 0.0, a2: -140.0, a3: 125, a4: 0.0, a5: 82.0, a6: -169}'
        ], check=True)
        print("✅ 초기 위치 명령이 성공적으로 전송되었습니다.")
    except subprocess.CalledProcessError as e:
        print("❌ 서비스 호출 중 오류 발생:", e)
    except KeyboardInterrupt:
        print("\n⏹ 사용자에 의해 중단되었습니다.")

if __name__ == "__main__":
    call_kuka_initial_pose()

