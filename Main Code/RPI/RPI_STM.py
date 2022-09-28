###################################################
#
# Note:
# To test RPI_Arduino code without MultiProcess, 
# uncomment all the commented lines in the code
#
###################################################

import serial
import time

from config import SERIAL_PORT, BAUD_RATE, LOCALE
from colorama import *

init(autoreset=True)


class STMComm:
    def __init__(self):
        self.port = SERIAL_PORT
        self.baud_rate = BAUD_RATE
        self.stm_connection = None

    def connect_stm(self):
        print(Fore.LIGHTYELLOW_EX + '[ARD-CONN] Waiting for serial connection...')
        while True:
            retry = False

            try:
                self.stm_connection = serial.Serial(self.port, self.baud_rate)

                if self.stm_connection is not None:
                    print(Fore.LIGHTGREEN_EX + '[ARD-CONN] Successfully connected with STM: %s' % str(self.stm_connection.name))
                    retry = False

            except Exception as e:
                print(Fore.RED + '[ARD-CONN ERROR] %s' % str(e))
                retry = True

            if not retry:
                break

            print(Fore.LIGHTYELLOW_EX + '[ARD-CONN] Retrying connection with Arduino...')
            time.sleep(1)

    def disconnect_stm(self):
        try:
            if self.stm_connection is not None:
                self.stm_connection.close()
                self.stm_connection = None
                print(Fore.LIGHTWHITE_EX + '[ARD-DCONN ERROR] Successfully closed connection')

        except Exception as e:
            print(Fore.RED + '[ARD-DCONN ERROR] %s' % str(e))

    def read_from_stm(self):
        try:
            self.stm_connection.flushInput()
            time.sleep(1)
            #get_message = self.stm_connection.readline().strip()
            get_message = self.stm_connection.read(5)
            # print('Transmission from Arduino:')
            # print('\t %s' % get_message)

            if len(get_message) > 0:
                return get_message

            return None

        except Exception as e:
            print(Fore.RED + '[ARD-READ ERROR] %s' % str(e))
            raise e

    def write_to_stm(self, message):
        try:
            if self.stm_connection is None:
                print(Fore.LIGHTYELLOW_EX + '[ARD-CONN] STM is not connected. Trying to connect...')
                self.connect_stm()

            self.stm_connection.flushOutput()
            time.sleep(0.1)
            # print('Transmitted to Arduino:')
            # print('\t %s' % message)
            self.stm_connection.write(message)
            print('Written to STM: %s' %str(message))

        except Exception as e:
            print(Fore.RED + '[ARD-WRITE Error] %s' % str(e))
            raise e


if __name__ == '__main__':
     ser = STMComm()
     ser.__init__()
     ser.connect_stm()
     while True:
         try:
             ser.write_to_stm(b'w')
             break
         except KeyboardInterrupt:
             print('Serial Communication Interrupted.')
             ser.disconnect_stm()
             break
