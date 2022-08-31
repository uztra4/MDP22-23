import struct
import socket
import typing as T

def send_msg(sock: socket.socket, msg: bytes):
    assert isinstance(msg, bytes), "Expected msg to be of type bytes"

    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)

def recv_msg(sock: socket.socket) -> T.Optional[bytes]:
    # Read message length and unpack it into an integer
    raw_msglen = _recvall(sock, 4)
    if not raw_msglen:
        raise IOError
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    data = _recvall(sock, msglen)
    if not data:
        raise IOError
    return data

def _recvall(sock: socket.socket, n: int) -> T.Optional[bytes]:
    # Helper function to recv n bytes or return None if EOF is hit
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data
