from multiprocessing import Process, Value, Queue, Manager, Lock
import string
import time
from datetime import datetime

# from RPI_PC import PcComm
from RPI_Android import AndroidComm
from RPI_STM import STMComm
from RPI_PC import PcComm
# from IP import ImageProcessor
from config import IMAGE_PROCESSING_SERVER_URL_LIST, MESSAGE_SEPARATOR, NEWLINE, LOCALE, IMAGE_PROCESSING_MODE
from colorama import *
from img_rec import *
from sendtopc import *


#from picamera import PiCamera
#import socket
# import cv2
#import imagezmq
#from imutils.video import VideoStream

# import socket
import time
from imutils.video import VideoStream
# import imagezmq
# from picamera import PiCamera
# from picamera.array import PiRGBArray

init(autoreset=True)

class MultiProcessCommunication:
    def __init__(self):

        self.IMAGE_PROCESSING_SERVER_URL = ''

     #    if IMAGE_PROCESSING_MODE:
     #        while True:
     #            try:
     #                self.ir_auth = int(input('[MultiProcess-IR SETUP] Enter IR setup id: '))
     #                if self.ir_auth == 0:
     #                    print("test")
     #                    self.IMAGE_PROCESSING_SERVER_URL = IMAGE_PROCESSING_SERVER_URL_LIST[self.ir_auth]
     #                    print(Fore.LIGHTMAGENTA_EX + '[MultiProcess-IR SETUP] IR is setup for EL (%s)' % str(self.IMAGE_PROCESSING_SERVER_URL))
     #                    break
     #                elif self.ir_auth == 1:
     #                    print(Fore.LIGHTMAGENTA_EX + '[MultiProcess-IR SETUP] Skipping IR setup')
     #                    break
     #                else:
     #                    print(Fore.RED + '[MultiProcess-IR SETUP ERROR] IR setup id not found')
     #            except:
     #                print(Fore.RED + '[MultiProcess-IR SETUP ERROR] Invalid Input')

        self.stm = STMComm()
        self.algorithm = PcComm()
        self.android = AndroidComm()

        self.manager = Manager()
     #    self.IMAGE_LIST = self.manager.list()

        self.message_queue = self.manager.Queue()
        self.to_android_message_queue = self.manager.Queue()

        lock = Lock()
        moveNext = Value('i', 1) # for STM to tell RPI it is ready for next command.
        self.read_stm_process = Process(target=self._read_stm, args=(moveNext,lock))
        self.read_algorithm_process = Process(target=self._read_algorithm)
        self.read_android_process = Process(target=self._read_android)

        self.write_process = Process(target=self._write_target, args=(moveNext,lock))
        self.write_android_process = Process(target=self._write_android)
        print(Fore.LIGHTGREEN_EX + '[MultiProcess] MultiProcessing initialized')

        self.dropped_connection = Value('i', 0)



        self.sender = None
     #    self.image_process = Process(target=self._process_pic)
     #    self.image_queue = self.manager.Queue()

    def start(self):
        try:
            # Connect to STM, Algorithm (PC) and Android
            self.stm.connect_stm()
            self.algorithm.connect_PC()
            self.android.connect_android()

            # Start the process to read and write to various modules (STM, Algorithm [PC] and Android)
            self.read_stm_process.start()
            self.read_algorithm_process.start()
            self.read_android_process.start()
            # message_list = ["ALG", b'15,55,-90,0']
            # self.message_queue.put_nowait(self._format_for(message_list[0], message_list[1]))
            self.write_process.start()
            self.write_android_process.start()
            startComms_dt = datetime.now().strftime('%d-%b-%Y %H:%M%S')
            print(Fore.LIGHTGREEN_EX + str(startComms_dt) + '| [MultiProcess] Communications started. Reading from STM, Algorithm & Android')

          #   # Start the Image Rec process
          #   self.image_process.start()
          #   startImage_dt = datetime.now().strftime('%d-%b-%Y %H:%M%S')
          #   print(Fore.LIGHTGREEN_EX + str(startImage_dt) + '| [MultiProcess-IR] Image Processing Mode is ON')

        except Exception as e:
            print(Fore.RED + '[MultiProcess-START ERROR] %s' % str(e))
            raise e

        self._allow_reconnection()

    def _allow_reconnection(self):
        while True:
            try:
                if not self.read_stm_process.is_alive():
                    self._reconnect_stm()

                if not self.read_algorithm_process.is_alive():
                    self._reconnect_algorithm()

                if not self.read_android_process.is_alive():
                    self._reconnect_android()

                if not self.write_process.is_alive():
                    if self.dropped_connection.value == 0:
                        self._reconnect_stm()
                    elif self.dropped_connection.value == 1:
                        self._reconnect_algorithm()

                if not self.write_android_process.is_alive():
                    self._reconnect_android()


            except Exception as e:
                print(Fore.RED + '[MultiProcess-RECONN ERROR] %s' % str(e))
                raise e

    def _reconnect_stm(self):
        self.stm.disconnect_stm()

        self.read_stm_process.terminate()
        self.write_process.terminate()
        self.write_android_process.terminate()

        self.stm.connect_stm()

        self.read_stm_process = Process(target=self._read_stm)
        self.read_stm_process.start()

        self.write_process = Process(target=self._write_target)
        self.write_process.start()

        self.write_android_process = Process(target=self._write_android)
        self.write_android_process.start()

        print(Fore.LIGHTGREEN_EX + '[MultiProcess-RECONN] Reconnected to STM')

    def _reconnect_algorithm(self):
        self.algorithm.disconnect_PC()

        self.read_algorithm_process.terminate()
        self.write_process.terminate()
        self.write_android_process.terminate()

        self.algorithm.connect_PC()

        self.read_algorithm_process = Process(target=self._read_algorithm)
        self.read_algorithm_process.start()

        self.write_process = Process(target=self._write_target)
        self.write_process.start()

        self.write_android_process = Process(target=self._write_android)
        self.write_android_process.start()

        print(Fore.LIGHTGREEN_EX + '[MultiProcess-RECONN] Reconnected to Algorithm')

    def _reconnect_android(self):
        self.android.disconnect_android()

        self.read_android_process.terminate()
        self.write_process.terminate()
        self.write_android_process.terminate()

        self.android.connect_android()

        self.read_android_process = Process(target=self._read_android)
        self.read_android_process.start()

        self.write_process = Process(target=self._write_target)
        self.write_process.start()

        self.write_android_process = Process(target=self._write_android)
        self.write_android_process.start()

        print(Fore.LIGHTGREEN_EX + '[MultiProcess-RECONN] Reconnected to Android')

    def _format_for(self, target, message):
        # Function to return a dictionary containing the target and the message
        return {
            'target': target,
            'payload': message,
        }

    def _read_stm(self, moveNext, lock):
        while True:
            try:
                raw_message = self.stm.read_from_stm()

                if raw_message is None:
                    print(Fore.LIGHTBLUE_EX + 'No Message from STM')
                    continue

                #raw_message_list = raw_message.decode().splitlines()
                raw_message_list = raw_message.decode()

                #for pre_message_list in raw_message_list:
                if len(raw_message_list) != 0:

                    message_list = raw_message_list.split(MESSAGE_SEPARATOR, 1)

                    if message_list[0] == 'AND':
                        print(Fore.LIGHTCYAN_EX + 'STM > %s , %s' % (str(message_list[0]), str(message_list[1])))
                        self.to_android_message_queue.put_nowait(message_list[1].encode(LOCALE))

                    elif message_list[0] == 'ALG':
                        print(Fore.LIGHTCYAN_EX + 'STM > %s , %s' % (str(message_list[0]), str(message_list[1])))
                        self.message_queue.put_nowait(self._format_for(message_list[0], message_list[1].encode(LOCALE)))

                    elif message_list[0] == 'RPI' and message_list[1] == 'd':
                        with lock:
                            moveNext.value += 1
                        print("execution from STM IS DONE")
                        

                    elif message_list[0] == 'RPI' and message_list[1] == 's':
                        print('scanning image...')
                        label = imgRec() #In the format of OBSTACLE_NUM:IMAGE_ID
                        label = label.split(MESSAGE_SEPARATOR, 1)

                        #send image id to AND``
                        if label[1]!='null' and label[1]!='bullseye':
                            stringToAND = "TARGET," + str(label[0]) + ',' + str(label[1])
                            print(Fore.LIGHTCYAN_EX + 'RPI > AND , %s' % (stringToAND))
                            self.message_queue.put_nowait(self._format_for("AND", stringToAND.encode(LOCALE)))

                        #send image id to ALG
                        print(Fore.LIGHTCYAN_EX + 'RPI > ALG , %s' % (str(label[1])))
                        self.message_queue.put_nowait(self._format_for("ALG", str(label[1]).encode(LOCALE)))


                    else:
                        # Printing message without proper message format on RPi terminal for STM sub-team to debug
                        print(Fore.LIGHTBLUE_EX + '[Debug] Message from STM: %s' % str(message_list))

            except Exception as e:
                print(Fore.RED + '[MultiProcess-READ-STM ERROR] %s' % str(e))
                break

    def _read_algorithm(self):
        #picam = VideoStream(usePicamera=True).start()
        while True:
            try:
                raw_message = self.algorithm.read_from_pc()
                if raw_message is None:
                    continue

                raw_message_list = raw_message.decode().splitlines()

                for pre_message_list in raw_message_list:
                    if len(pre_message_list) != 0:

                        message_list = pre_message_list.split(MESSAGE_SEPARATOR, 1)

                        # Sending STOPPED message to AND
                        if message_list[0] == 'RPI':
                            if message_list[1] == 'STOPPED':
                                statusSTM = "STATUS," + message_list[1]
                                print(Fore.LIGHTCYAN_EX + 'Algo > AND , %s' % (statusSTM))
                                self.message_queue.put_nowait(self._format_for("AND", statusSTM.encode(LOCALE)))

                        elif message_list[0] == 'AND':
                            print(Fore.LIGHTCYAN_EX + 'Algo > %s , %s' % (str(message_list[0]), str(message_list[1])))
                            self.to_android_message_queue.put_nowait(message_list[1].encode(LOCALE))

                        elif message_list[0] == 'STM':
                            # Send to STM for movements
                            print(Fore.LIGHTCYAN_EX + 'Algo > %s , %s' % (str(message_list[0]), str(message_list[1])))
                            for char in message_list[1]:
                                self.message_queue.put_nowait(self._format_for(message_list[0], char.encode(LOCALE)))

                            appendedString = "MOVE," + str(message_list[1])
                            # Send to AND to update movements
                            print(Fore.LIGHTCYAN_EX + 'Algo > AND, %s' % (appendedString))
                            self.message_queue.put_nowait(self._format_for("AND", appendedString.encode(LOCALE)))

                        else:
                            print(Fore.LIGHTBLUE_EX + '[Debug] Message from ALGO: %s' % str(message_list))

            except Exception as e:
                print(Fore.RED + '[MultiProcess-READ-ALG ERROR] %s' % str(e))
                break

    def _read_android(self):
        while True:
            try:
                raw_message = self.android.read_from_android()

                if raw_message is None:
                    continue
                raw_message_list = raw_message.decode().splitlines()

                for pre_message_list in raw_message_list:
                    if len(pre_message_list) != 0:

                        message_list = pre_message_list.split(MESSAGE_SEPARATOR, 1)

                        if message_list[0] == 'ALG':

                            coordinates = message_list[1].split(';',8)
                            stringCoordinates = ""

                            #sending coordinates one obstacle by one obstacle
                            for i in range(len(coordinates)):
                                if i == 0:
                                    stringCoordinates = coordinates[0]
                                else:
                                    if len(coordinates[i]) > 1:
                                        stringCoordinates = stringCoordinates + ',' + coordinates[i]

                            print(Fore.LIGHTCYAN_EX + 'Android > %s , %s' % (str(message_list[0]), str(stringCoordinates)))
                            assert isinstance(stringCoordinates, object)
                            self.message_queue.put_nowait(self._format_for(message_list[0], stringCoordinates.encode(LOCALE)))

                        else:
                            print(Fore.LIGHTBLUE_EX + '[Debug] Message from AND: %s' % str(message_list))
                            self.message_queue.put_nowait(self._format_for(message_list[0], message_list[1].encode(LOCALE)))

            except Exception as e:
                print(Fore.RED + '[MultiProcess-READ-AND ERROR] %s' % str(e))
                break


    def _write_target(self, moveNext, lock):
        while True:
            target = None
            try:
                if not self.message_queue.empty():
                    message = self.message_queue.get_nowait()
                    target, payload = message['target'], message['payload']
                    if target == 'ALG':
                        self.algorithm.write_to_pc(payload)
                    elif target == 'STM':
                        self.stm.write_to_stm(payload)
                        with lock:
                            if(payload == b'n'):
                                moveNext.value -= 1
                        while(moveNext.value == 0):
                            continue
                        print("write message to stm")
                    elif target == 'AND':
                        self.android.write_to_android(payload)
            except Exception as e:
                print(Fore.RED + '[MultiProcess-WRITE-%s ERROR] %s' % (str(target), str(e)))

                if target == 'STM':
                    self.dropped_connection.value = 0

                elif target == 'ALG':
                    self.dropped_connection.value = 1

                break

    def _write_android(self):
        while True:
            try:
                if not self.to_android_message_queue.empty():
                    message = self.to_android_message_queue.get_nowait()
                    self.android.write_to_android(message)
            except Exception as e:
                print(Fore.RED + '[MultiProcess-WRITE-AND ERROR] %s' % str(e))
                break

    def _process_pic(self): # %terminate%
        self.sender = imagezmq.ImageSender(connect_to=self.IMAGE_PROCESSING_SERVER_URL)

        while True:
            try:
                if not self.image_queue.empty():
                    IMAGE_OBS_ID = self.image_queue.get_nowait()

                    self.rpi_name = socket.gethostname() # send RPi hostname with each image
                    self.camera = PiCamera(resolution=(640, 640)) #max resolution 2592,1944
                    self.rawCapture = PiRGBArray(self.camera)

                    #time.sleep(0.1)
                    self.camera.capture(self.rawCapture, format="bgr")
                    image = self.rawCapture.array
                    self.rawCapture.truncate(0)
            #         self.sender.send_image(rpi_name, image)
                    reply = self.sender.send_image(self.rpi_name, image)
                    print('Image sent')
                    reply1 = reply.decode()

                    print(reply1)
                    self.camera.close()

                    if int(reply1) > 0 and int(reply1) < 31:
                        id_msg = 'TARGET|'+ IMAGE_OBS_ID + '|' + str(reply1)
                        self.to_android_message_queue.put_nowait(id_msg.encode(LOCALE))
                        print('ID sent to Android')
                    #return reply

            except Exception as e:
                print(Fore.RED + '[MultiProcess-PROCESS-IMG ERROR] %s' % str(e))


def init():
    try:
        multi = MultiProcessCommunication()
        multi.start()
    except Exception as err:
        print(Fore.RED + '[Main.py ERROR] {}'.format(str(err)))

if __name__ == '__main__':
    init()
