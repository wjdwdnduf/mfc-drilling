# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modifications (2026): added an optional socket timeout and context-manager
# support so callers can bound connect/recv and close deterministically.

from typing import Optional, Tuple
import socket


Address = Tuple[str, int]


class TcpClient:
    def __init__(self, address: Address, timeout: Optional[float] = None):
        self._address: Address = address
        self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if timeout is not None:
            self._socket.settimeout(timeout)

    def connect(self) -> None:
        self._socket.connect(self._address)

    def sendall(self, data: bytes) -> None:
        self._socket.sendall(data)

    def recv(self, bufsize: int = 1024) -> bytes:
        return self._socket.recv(bufsize)

    def close(self) -> None:
        try:
            self._socket.close()
        except OSError:
            pass

    def __enter__(self) -> "TcpClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
