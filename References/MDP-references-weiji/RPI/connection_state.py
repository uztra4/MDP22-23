import enum

class ConnectionState(enum.Enum):
    STARTUP = 0
    CONNECTED = 1
    DISCONNECTED = 2
