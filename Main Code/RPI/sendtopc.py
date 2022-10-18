import struct
import socket
import sys
import threading
import time


# --- constants ---
HOST = '192.168.45.208'   # (local or external) address IP of remote server
PORT = 5001 # (local or external) port of remote server

def sendImgToPC():
    
    def sender(s):
        
        f = open(f'/home/mdp2022/shared/image/image1.jpg','rb')

        print('Sending...')
        
        data = f.read(4096)
        while data != bytes(''.encode()):
            s.sendall(data)
            data = f.read(4096)
        
        print("IMG sent")

        #time.sleep(3)
        #print('[sendtopc.py] close socket')
        # s.close()

    try:
        # --- create socket ---
        print('[sendtopc.py] create socket')
        s = socket.socket()         
        print('[sendtopc.py] connecting:', (HOST, PORT))
        s.connect((HOST, PORT))
        print('[sendtopc.py] connected')
        
        # --- send data ---
        # sendData = threading.Thread(target=sender, args=(s,))
        # sendData.start()

        sender(s)
        label = s.recv(1024).decode()
        return label

    except Exception as e:
        print(e)
    except KeyboardInterrupt as e:
        print(e)
    except:
        print(sys.exc_info())

def connect_to_server():
    try:
        s = socket.socket()
        s.connect((HOST, PORT))

        return s.recv(1024).decode()
    except socket.error:
        print("Could not connect")
