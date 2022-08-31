import serial
import time

class STM32(object):

    # Initialise the connection with STM32F Board
    # Check the connection port again, baud rate = 115200
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.baudrate = 115200
        self.is_connected = False

    def connect(self):
        while not self.is_connected:
            try:
                print("Establishing connection with STM32 Board ...")

                # Create a Serial instance named 'ser'
                self.ser = serial.Serial(self.port, self.baudrate, timeout = 20)
                # Check if the serial instance is open
                if (self.ser.is_open):
                    print("[SUCCESSFUL CONNECTION]: Successfully established connection with STM32 Board.")
                    self.is_connected = True

            except Exception as e:
                print(f"[ERROR] Unable to establish connection with STM32 Board: {e}")
                # Retry to connect
                print("Retrying to connect with STM32 Board ...")
                time.sleep(1)

    def disconnect(self):
        try:
            if (self.ser): #Check there is a serial instance created
                print("STM32: Disconnecting from STM32 Board ...")
                self.ser.close()
                self.is_connected = False
        except Exception as e:
            print(f"[ERROR]: Unable to disconnect from STM32 Board: {str(e)}") 

    # Getter method - to check if connection with STM32F board is established
    def get_is_connected(self):
        return self.is_connected

    # Read and process message
    def read(self):
        try:
            data = self.ser.read(1).decode("UTF-8")
            if data == '': # No data read
                return "No reply"
            print(f"[FROM STM32] {data}")
            return data
        except Exception as e:
            print(f"[ERROR] STM32 Board read error: {str(e)}")


    # Write message
    def write(self, message):
        try:
            # Ensure connection is established before sending a message
            if self.is_connected:
                self.ser.write(message.encode('UTF-8'))
                print(f"[SENT TO STM32]: {message}")
            else:
                print("[Error]  Connection with STM32 board is not established")
        except Exception as e:
            print("[Error] Unable to send message from STM32: %s" % str(e))
