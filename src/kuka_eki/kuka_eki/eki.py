# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

from typing import Union
from kuka_eki.krl import RobotCommand, RobotState
from kuka_eki.tcp_client import TcpClient
from kuka_eki.krl import Axis, Pos, CommandType
import xml.etree.ElementTree as ET

class EkiMotionClient:
    MOTION_PORT: int = 54600

    def __init__(self, ip_address: str, port: int | None = None) -> None:
        self._tcp_client = TcpClient((ip_address, port or self.MOTION_PORT))

    def connect(self) -> None:
        self._tcp_client.connect()

    def ptp(self, target: Union[Axis, Pos], max_velocity_scaling=1.0) -> None:
        command_type: CommandType
        if isinstance(target, Axis):
            command_type = CommandType.PTP_AXIS
        elif isinstance(target, Pos):
            command_type = CommandType.PTP_CART
        else:
            raise TypeError("Expected argument of type Axis or Pos")
        command: RobotCommand = RobotCommand(command_type, target, max_velocity_scaling)
        self._tcp_client.sendall(command.to_xml())

    def ptp_rel(self, target: Axis, max_velocity_scaling: float = 1.0) -> None:
        if not isinstance(target, Axis):
            raise TypeError("Expected argument of Axis")
        command: RobotCommand = RobotCommand(
            CommandType.PTP_AXIS_REL, target, max_velocity_scaling
        )
        self._tcp_client.sendall(command.to_xml())

    def lin(self, target: Pos, max_velocity_scaling=1.0) -> None:
        if not isinstance(target, Pos):
            raise TypeError("Expected argument of Pos")
        command: RobotCommand = RobotCommand(
            CommandType.LIN_CART, target, max_velocity_scaling
        )
        self._tcp_client.sendall(command.to_xml())

    def lin_rel(self, target: Pos, max_velocity_scaling=1.0) -> None:
        if not isinstance(target, Pos):
            raise TypeError("Expected argument of Pos")
        command: RobotCommand = RobotCommand(
            CommandType.LIN_CART_REL, target, max_velocity_scaling
        )
        self._tcp_client.sendall(command.to_xml())


class EkiStateClient:
    STATE_PORT: int = 54602

    def __init__(self, ip_address: str, port: int | None = None) -> None:
        self._tcp_client: TcpClient = TcpClient((ip_address, port or self.STATE_PORT))
        self._buffer = b''  # 수신된 데이터를 저장할 버퍼를 초기화합니다.

    def connect(self) -> None:
        self._tcp_client.connect()

    def state(self) -> RobotState:
        try:
            # 소켓으로부터 데이터를 읽어옵니다.
            data: bytes = self._tcp_client.recv(1024)
            if not data:
                return RobotState()
            # 수신된 데이터를 버퍼에 추가합니다.
            self._buffer += data

            # 버퍼에서 완전한 XML 메시지를 추출합니다.
            while True:
                start = self._buffer.find(b'<RobotState>')
                end = self._buffer.find(b'</RobotState>', start)
                if start != -1 and end != -1:
                    end += len(b'</RobotState>')
                    xml_message = self._buffer[start:end]
                    # 추출한 메시지를 버퍼에서 제거합니다.
                    self._buffer = self._buffer[end:]
                    # XML 메시지를 파싱하여 RobotState 객체를 반환합니다.
                    return RobotState.from_xml(xml_message)
                else:
                    # 완전한 메시지가 없을 경우 다음 recv() 호출까지 기다립니다.
                    break
            # 완전한 메시지를 처리하지 못한 경우 빈 RobotState를 반환합니다.
            return RobotState()
        except ET.ParseError as e:
            print(f"XML 파싱 오류: {e}")
            return RobotState()
        except Exception as e:
            print(f"로봇 상태 수신 중 예기치 않은 오류: {e}")
            return RobotState()
