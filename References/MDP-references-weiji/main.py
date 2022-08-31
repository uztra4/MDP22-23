"""
This module will provide the 'brains' of the entire project, providing an interface 
between the STM, Android and RPI layers 

Image recognition script will settle its own recognition

This controls only the pathfinding?

STM <-> Android 
RPI is server

"""

import multiprocessing
import time
import logging
import traceback
import json
from dataclasses import dataclass
from enum import Enum

from Image_Recognition import detect_V2
from RPI import bluetooth_test
from RPI import rpi_camera_stream

logging.basicConfig(level=logging.DEBUG)

class UserInputInterface:
    @classmethod
    def open_connection(cls):
        return cls.BluetoothInterface.open_bluetooth_server();
    
    def send_msg(cls, msg):
        return cls.BluetoothInterface.send_msg(msg)

    def receive_messages(cls, *args, **kwargs):
        """Get the incoming messages from the android app
        are the messages stored in some buffer?
        does this function return messages from the buffer?
        """
        pass

    def parse_message(cls, message: str):
        '''	
        Converts incoming messages from string JSON format to useful outputs
        
        Parameters
        ----------
        message: str
            Incoming message from Android
        
        Returns
        -------
        ?
        '''
        try:
            message = json.loads(message)
        except ValueError as e:
            logging.debug(e)
            logging.debug(f'Could not parse message {message}')

        for key in message.keys():
            msg_val = message[key]
            if strings_match(key, 'test'):
                cls.handle_test_string(msg_val)
            elif strings_match(key, 'move'):
                RobotInterface.handle_move(msg_val)
            elif strings_match(key, 'arena'):
                RobotController.Pathfinder.add_or_update_obstacle(msg_val)
            elif strings_match(key, 'start_location'):
                RobotController.Pathfinder.update_start_location(msg_val)
            elif strings_match(key, 'final_location'):
                RobotController.Pathfinder.update_final_location(msg_val)
            elif strings_match(key, 'path'):
                RobotController.Pathfinder.set_fixed_path(msg_val)
            else:
                logging.debug(f'Key {key} in command from Android app not recognised')


    def handle_test_string(string: str):
        pass

    class BluetoothInterface:
        def open_bluetooth_server():
            pass

        def send_bluetooth_msg_to_android():
            pass


class RobotInterface:
    class ValidCommands(Enum):
        LEFT    = 'LEFT'
        RIGHT   = 'RIGHT'
        FORWARD = 'FORWARD'
        REVERSE = 'REVERSE'

    def handle_move_command(cls, command:str): 
        """ Parse a move command to be executed by the robot, only allowing a set of inputs
        Does not raise Exception if invalid command
        returns
        ========
        True if input is a valid command, False if invalid command 
        """
        if not isinstance(command, str): 
            traceback.print_stack()
            logging.debug('Move command passed to function is not a string')
            return False

        command = command.upper().strip()
        if command in cls.valid_commands:
            if command ==  cls.ValidCommands.FORWARD:
                cls.move_forward()
            elif command ==  cls.ValidCommands.REVERSE:
                cls.move_reverse()
            elif command ==  cls.ValidCommands.LEFT:
                cls.move_left()
            elif command ==  cls.ValidCommands.RIGHT:
                cls.move_right()
        else:
            return False


    def move_forward(cls):
        # data format not finalized
        data = {
            'move': 'up'
        }
        cls._send_msg_to_STM('move up')
    def move_left(cls):
        pass
    def move_right(cls):
        pass
    def move_reverse(cls):
        pass
    ...

    def _send_msg_to_STM():
        pass

    def config_STM_on_current_settings():
        """ Use the 2 mins given at the start to configure the STM to the current battery levels, motor power etc."""
        pass


class ImageRecognizer:
    def get_image_from_camera():
        pass

    def call_image_recognition():
        return detect_V2.main()


def strings_match(str1:str, str2:str):
    return str1.lower().strip() == str2.lower().strip()


def wait():
    pass


def main():
    # start up script to ensure all connections are active
    bt_server = UserInputInterface.open_connection()
    while not bt_server.connection_established():
        wait()

    sr_link = RobotInterface.open_connection()
    while not sr_link.connection_established():
        wait()

    RobotInterface.config_STM_on_current_settings()

    # CODE TO ADD NOW
    

    # manager = multiprocessing.Manager()

    # messages_from_android = manager.Queue()
    # flags = manager.dict({
    #     'pathfind_running': True
    #     })

    # pool = multiprocessing.Pool()

    # android_handler_process = pool.apply_async(UserInputInterface.receive_messages, args=(messages_from_android, flags))
    # stm_handler_process     = pool.apply_async(RobotController.start_processing, arg=())

if __name__ == "__main__":
    pass